#include "FEMObject.h"
#include "modules/mesh_converter/MeshConverter.h"
#include <simulation/ElasticMaterial.h>
#include <simulation/SimulationObjectVisitor.h>
#include "simulation/SimulationUtils.h"
#include "simulation/constraints/Truncation.h"
#include <QDebug>
#include <iostream>
#include <scene/data/geometric/Polygon3D.h>
#include <scene/data/geometric/Polygon3DTopology.h>
#include <scene/data/references/GeometricPointRefVisitor.h>
#include <scene/data/references/GeometricVertexRef.h>
#include <simulation/constraints/Truncation.h>
#include <simulation/references/SimulationPointRef.h>
#include <times/timing.h>

// FEMObject without initialPositions

using namespace Eigen;

FEMObject::FEMObject(
        Domain* domain,
        std::shared_ptr<Polygon3D> poly3)
    : SimulationObject(domain, SimulationObject::Type::FEM_OBJECT)
    , mPoly3(poly3)
    , mPositions(poly3->getPositions())
    , mK(static_cast<Eigen::Index>(mPositions.size()),
         static_cast<Eigen::Index>(mPositions.size()))
    , mKCorot(static_cast<Eigen::Index>(mPositions.size()),
              static_cast<Eigen::Index>(mPositions.size()))
    , mAnalyzePatternNecessary(true)
{
    mInitialPositions.resize(mPositions.size());
    std::copy(mPositions.begin(), mPositions.end(), mInitialPositions.begin());

    mVelocities.resize(mPositions.size());
    for (size_t i = 0; i < mPositions.size(); ++i)
    {
        mVelocities[i] = Vector::Zero();
    }

    // Initialize finite elements
    double v = 0.45;
    double E = 5e+3;
    ElasticMaterial material;
    material.setFromYoungsPoisson(E, v);

    const Cells& cells = poly3->getTopology3D().getCellIds();
    mFiniteElements.reserve(cells.size());
    for (unsigned int i = 0; i < cells.size(); ++i)
    {
        mFiniteElements.push_back(FiniteElement(this, cells[i], material));
    }

    unsigned int nVertices = static_cast<unsigned int>(mPositions.size());

    // Initialize vectors
    mPositionsPrevious = mPositions;
    mVelocitiesPrevious = mVelocities;
    mDisplacementsPrevious.resize(mPositions.size());
    SimulationUtils::resizeVectorWithZeros(mDisplacements, nVertices);
    SimulationUtils::resizeVectorWithZeros(mForcesEl, nVertices);
    SimulationUtils::resizeVectorWithZeros(mForcesExt, nVertices);
    SimulationUtils::resizeVectorWithZeros(mDeltaV, nVertices);

    // Initialize other FEM values
    mM = SparseMatrix<double>(nVertices*3, nVertices*3);
    mK.setZero();
    mKCorot.setZero();
    mM.setZero();

    // init vertex masses
    std::vector<int> nCellsPerVertex;
    nCellsPerVertex.resize(mPositions.size());
    mMasses.resize(mPositions.size());
    for (size_t i = 0; i < mPositions.size(); ++i)
    {
        mMasses[i] = 0.0;
        nCellsPerVertex[i] = 0;
    }
    for (FiniteElement& fe : mFiniteElements)
    {
        fe.initialize();
        for (size_t i = 0; i < 4; ++i)
        {
            unsigned int index = fe.getCell().at(i);
            mMasses[index] += fe.getM().at(i);
            nCellsPerVertex[index]++;
        }
    }

    // set initial positions
    std::copy(mPositions.begin(), mPositions.end(), mInitialPositions.begin());

    mTruncation = std::make_shared<Truncation>();
}

FEMObject::~FEMObject()
{

}

void FEMObject::accept(SimulationObjectVisitor& visitor)
{
    visitor.visit(*this);
}

void FEMObject::applyImpulse(SimulationPointRef& ref, const Vector& impulse)
{
    class ApplyImpulseVisitor : public GeometricPointRefVisitor
    {
    public:
        ApplyImpulseVisitor(FEMObject& _femObj, const Eigen::Vector& _impulse)
            : femObj(_femObj)
            , impulse(_impulse)
        {
        }

        virtual void visit(GeometricVertexRef& ref)
        {
            femObj.applyImpulse(ref.getIndex(), impulse);
        }

        virtual void visit(PolygonVectorRef& /*ref*/)
        {
            // nothing to do here
        }

        FEMObject& femObj;
        const Eigen::Vector& impulse;
    } visitor(*this, impulse);

    ref.getGeometricPointRef()->accept(visitor);
}

void FEMObject::applyForce(SimulationPointRef& ref, const Vector& force)
{
    class ApplyForceVisitor : public GeometricPointRefVisitor
    {
    public:
        ApplyForceVisitor(FEMObject& _femObj, const Eigen::Vector& _force)
            : femObj(_femObj)
            , force(_force)
        {
        }

        virtual void visit(GeometricVertexRef& ref)
        {
            femObj.applyForce(ref.getIndex(), force);
        }

        virtual void visit(PolygonVectorRef& /*ref*/)
        {
            // nothing to do here
        }

        FEMObject& femObj;
        const Eigen::Vector& force;
    } visitor(*this, force);

    ref.getGeometricPointRef()->accept(visitor);
}

ID FEMObject::getId() const
{
    return mId;
}

void FEMObject::setId(ID id)
{
    mId = id;
}

void FEMObject::initializeFEM()
{
    // initializes getMassMatrix(), getStiffnessMatirx()
    for (FiniteElement& fe : mFiniteElements)
        fe.initialize();
    initializeStiffnessMatrix();

    updateFEM(false);
//    updateStiffnessMatrix(false);
    updateMassMatrix();
}

void FEMObject::updateDisplacements()
{
    Index size = static_cast<Index>(mDisplacements.size());
    VectorXd::Map(mDisplacements[0].data(), size * 3)
            =
            VectorXd::Map(mPositions[0].data(), size * 3) -
            VectorXd::Map(mInitialPositions[0].data(), size * 3);
}

void FEMObject::updatePositions()
{
    Index size = static_cast<Index>(mPositions.size());
    VectorXd::Map(mPositions[0].data(), size * 3)
            =
            VectorXd::Map(mInitialPositions[0].data(), size * 3) +
            VectorXd::Map(mDisplacements[0].data(), size * 3);
}

void FEMObject::updateFEM(bool corotated)
{
    START_TIMING_SIMULATION("FEMObject::updateFEM()");

    updateStiffnessMatrix(corotated);
    updateElasticForces();

    unsigned int size = static_cast<unsigned int>(mPositions.size());
    VectorXd::Map(mPositionsPrevious[0].data(), size * 3) =
            VectorXd::Map(mPositions[0].data(), size * 3);
    VectorXd::Map(mDisplacementsPrevious[0].data(), size * 3) =
            VectorXd::Map(mPositionsPrevious[0].data(), size * 3) -
            VectorXd::Map(mInitialPositions[0].data(), size * 3);
    VectorXd::Map(mVelocitiesPrevious[0].data(), size * 3) =
            VectorXd::Map(mVelocities[0].data(), size * 3);

    STOP_TIMING_SIMULATION;
}

void FEMObject::solveFEMExplicitly(double timeStep, bool corotated)
{
    unsigned int size = static_cast<unsigned int>(mVelocities.size());
    VectorXd v = VectorXd::Map(mVelocities[0].data(), size * 3);
    VectorXd y = VectorXd::Map(mPositions[0].data(), size * 3);
    VectorXd f = VectorXd::Map(mForcesExt[0].data(), size * 3)
            - VectorXd::Map(mForcesEl[0].data(), size * 3);

//    f = Truncation::createOriginal(f, mTruncatedVectorIds);
    for (ID i : mTruncation->getTruncatedVectorIds())
        for (ID j = 0; j < 3; ++j)
            f(static_cast<long>(3 * i + j)) = 0.0;

    VectorXd::Map(mVelocities[0].data(), size * 3) += timeStep * f;
    VectorXd::Map(mPositions[0].data(), size * 3) += timeStep * v;

    VectorXd::Map(mDisplacements[0].data(), size * 3) =
            VectorXd::Map(mPositions[0].data(), size * 3) -
            VectorXd::Map(mInitialPositions[0].data(), size * 3);
}

void FEMObject::solveFEM(double timeStep, bool corotated, bool firstStep)
{
    if (mTruncation->getTruncatedVectorIds().size() == mPositions.size())
        return;
    START_TIMING_SIMULATION("FEMObject::solveFEM()");

    // map velocities to eigen struct
    unsigned int size = static_cast<unsigned int>(mVelocities.size());

    Eigen::Map<VectorXd> v = VectorXd::Map(mVelocities[0].data(), size * 3);

    VectorXd f = VectorXd::Map(mForcesExt[0].data(), size * 3)
            - VectorXd::Map(mForcesEl[0].data(), size * 3);

    SparseMatrix<double>& K = mKCorot.getMatrix();
    if (!corotated)
        K = mK.getMatrix();
    VectorXd b;
//    if (firstStep)
//    {
        b = timeStep * (f - timeStep * K * v);
//    }
//    else
//    {
//        Eigen::Map<VectorXd> u = VectorXd::Map(mDisplacements[0].data(), size * 3);
//        Eigen::Map<VectorXd> uPrev = VectorXd::Map(mDisplacementsPrevious[0].data(), size * 3);
//        Eigen::Map<VectorXd> vPrev = VectorXd::Map(mVelocitiesPrevious[0].data(), size * 3);

//        b = mM * (vPrev - v) + timeStep * (f - K * (timeStep * v + uPrev - u));
//    }

    START_TIMING_SIMULATION("FEMObject::solveFEM()::linSolver");

    VectorXd sol;
    bool byRemoving = false;
    if (byRemoving)
    {
        if (firstStep)
        {
            START_TIMING_SIMULATION("FEMObject::solveFEM()::linSolver::calcAOriginal");
            SparseMatrix<double> AOriginal = mM + timeStep * timeStep * K;
            SparseMatrix<double> A;
            STOP_TIMING_SIMULATION;

            START_TIMING_SIMULATION("FEMObject::solveFEM()::linSolver::truncateByRemoving");
            mTruncation->truncateByRemoving(AOriginal, A);
            STOP_TIMING_SIMULATION;

            if (mAnalyzePatternNecessary)
            {
                START_TIMING_SIMULATION("FEMObject::solveFEM()::linSolver::analyze");
                mSolver.analyzePattern(A);
                mAnalyzePatternNecessary = false;
                STOP_TIMING_SIMULATION;
            }

            if (mSolver.info() != Eigen::Success)
                std::cerr << "Solver: analyzePattern failed.\n";

            START_TIMING_SIMULATION("FEMObject::solveFEM()::linSolver::factorize");
            mSolver.factorize(A);
            STOP_TIMING_SIMULATION;
        }

        if (mSolver.info() != Eigen::Success)
        {
            std::cerr << "Solver: factorize failed.\n";
        }

        VectorXd bCopy = b;
        mTruncation->truncateByRemoving(bCopy, b);

        START_TIMING_SIMULATION("FEMObject::solveFEM()::linSolver::solve");
        sol = mSolver.solve(b);
        STOP_TIMING_SIMULATION;

        sol = mTruncation->createOriginal(sol);
    }
    else
    {
        if (firstStep)
        {
            START_TIMING_SIMULATION("FEMObject::solveFEM()::linSolver::calcAOriginal");
            SparseMatrix<double> A = mM + timeStep * timeStep * K;
            STOP_TIMING_SIMULATION;

            if (mAnalyzePatternNecessary)
            {
                mTruncation->analyzePattern(A);
            }

            START_TIMING_SIMULATION("FEMObject::solveFEM()::linSolver::truncateBySettingZero");
            mTruncation->truncateBySettingZeroFast(A, b);
            STOP_TIMING_SIMULATION;

            if (mAnalyzePatternNecessary)
            {
                START_TIMING_SIMULATION("FEMObject::solveFEM()::linSolver::analyze");
                mSolver.analyzePattern(A);
                mAnalyzePatternNecessary = false;
                STOP_TIMING_SIMULATION;
            }

            if (mSolver.info() != Eigen::Success)
                std::cerr << "Solver: analyzePattern failed.\n";

            START_TIMING_SIMULATION("FEMObject::solveFEM()::linSolver::factorize");
            mSolver.factorize(A);
            STOP_TIMING_SIMULATION;
        }

        if (mSolver.info() != Eigen::Success)
        {
            std::cerr << "Solver: factorize failed.\n";
        }

        mTruncation->truncateBySettingZeroFast(b);

        START_TIMING_SIMULATION("FEMObject::solveFEM()::linSolver::solve");
        sol = mSolver.solve(b);
        STOP_TIMING_SIMULATION;
    }

    STOP_TIMING_SIMULATION; // FEMObject::solveFEM()::linSolver

    // update the velocities
    VectorXd::Map(mDeltaV[0].data(), size * 3) = sol;

    VectorXd::Map(mVelocities[0].data(), size * 3) +=
            VectorXd::Map(mDeltaV[0].data(), size * 3);

    integratePositions(timeStep);

    STOP_TIMING_SIMULATION; // FEMObject::solveFEM
}

void FEMObject::revertSolverStep()
{
    unsigned int size = static_cast<unsigned int>(mPositions.size());
    VectorXd::Map(mVelocities[0].data(), size * 3) -=
            VectorXd::Map(mDeltaV[0].data(), size * 3);
}

void FEMObject::integratePositions(double stepSize)
{
    unsigned int size = static_cast<unsigned int>(mPositions.size());

    VectorXd::Map(mPositionsPrevious[0].data(), size * 3) =
            VectorXd::Map(mPositions[0].data(), size * 3);
    VectorXd::Map(mDisplacementsPrevious[0].data(), size * 3) =
            VectorXd::Map(mPositionsPrevious[0].data(), size * 3) -
            VectorXd::Map(mInitialPositions[0].data(), size * 3);

    VectorXd::Map(mPositions[0].data(), size * 3) =
            VectorXd::Map(mPositionsPrevious[0].data(), size * 3) +
            stepSize * VectorXd::Map(mVelocities[0].data(), size * 3);
    VectorXd::Map(mDisplacements[0].data(), size * 3) =
            VectorXd::Map(mPositions[0].data(), size * 3) -
            VectorXd::Map(mInitialPositions[0].data(), size * 3);
}

void FEMObject::revertPositions()
{
    unsigned int size = static_cast<unsigned int>(mPositions.size());

    VectorXd::Map(mPositions[0].data(), size * 3) =
            VectorXd::Map(mPositionsPrevious[0].data(), size * 3);
    VectorXd::Map(mDisplacements[0].data(), size * 3) =
            VectorXd::Map(mPositions[0].data(), size * 3) -
            VectorXd::Map(mInitialPositions[0].data(), size * 3);
}

void FEMObject::updateGeometricData()
{
    mPoly3->getPositions() = mPositions;
    // This call is necessary to inform the other modules about
    // the change in position of the underlying Polygon3D that is
    // simulated here.
    mPoly3->update();
}

void FEMObject::applyImpulse(ID vertexIndex, const Vector& impulse)
{
    // calculate masses of each vertex (itearte over finite elements)
    if (mMasses[vertexIndex] > 0)
        mVelocities[vertexIndex] += 1 / mMasses[vertexIndex] * impulse;
    else
        std::cout << "error: mass is negative\n";
}

void FEMObject::applyForce(ID vertexIndex, const Vector& force)
{
    mForcesExt[vertexIndex] += force;
}

void FEMObject::addTrunctionIds(const std::vector<ID>& vectorIDs)
{
    mTruncation->addTruncationIds(vectorIDs);
    for (ID id : vectorIDs)
    {
        mVelocities[id].setZero();
    }

    mAnalyzePatternNecessary = true;
}

void FEMObject::removeTrunctionIds(const std::vector<ID>& vectorIDs)
{
    mTruncation->removeTruncationIds(vectorIDs);

    mAnalyzePatternNecessary = true;
}

void FEMObject::clearTruncation()
{
    mTruncation->clear();

    mAnalyzePatternNecessary = true;
}

const std::vector<ID>& FEMObject::getTruncatedVectorIds() const
{
    return mTruncation->getTruncatedVectorIds();
}

Vector& FEMObject::x(unsigned int i)
{
    return mInitialPositions[i];
}

Vector& FEMObject::y(unsigned int i)
{
    return mPositions[i];
}

Vector& FEMObject::u(unsigned int i)
{
    return mDisplacements[i];
}

Vector& FEMObject::v(unsigned int i)
{
    return mVelocities[i];
}

Vector& FEMObject::f_el(unsigned int i)
{
    return mForcesEl[i];
}

void FEMObject::setYoungsModulus(double youngsModulus)
{
    for (FiniteElement& fe : mFiniteElements)
    {
        fe.getMaterial().setFromYoungsPoisson(
                    youngsModulus,
                    fe.getMaterial().getPoissonRatio());
        fe.initialize();
    }
}

double FEMObject::getYoungsModulus()
{
    if (!mFiniteElements.empty())
    {
        return mFiniteElements[0].getMaterial().getYoungsModulus();
    }
    return 0.0;
}

void FEMObject::setPoissonRatio(double poissonRatio)
{
    for (FiniteElement& fe : mFiniteElements)
    {
        fe.getMaterial().setFromYoungsPoisson(
                    fe.getMaterial().getYoungsModulus(),
                    poissonRatio);
    }
}

double FEMObject::getPoissonRatio()
{
    if (!mFiniteElements.empty())
    {
        return mFiniteElements[0].getMaterial().getPoissonRatio();
    }
    return 0.0;
}

void FEMObject::setElasticMaterial(const ElasticMaterial& material)
{
    for (FiniteElement& fe : mFiniteElements)
    {
        fe.setMaterial(material);
    }
}

ElasticMaterial FEMObject::getElasticMaterial()
{
    if (!mFiniteElements.empty())
    {
        return mFiniteElements[0].getMaterial();
    }
    return ElasticMaterial();
}

Vectors& FEMObject::getInitialPositions()
{
    return mInitialPositions;
}

Vectors& FEMObject::getPositions()
{
    return mPositions;
}

const Vector& FEMObject::getPositionPrevious(size_t index) const
{
    return mPositionsPrevious[index];
}

Vectors& FEMObject::getVelocities()
{
    return mVelocities;
}

double FEMObject::getMass(ID vertexId)
{
    return mMasses[vertexId];
}

Vectors& FEMObject::getDisplacements()
{
    return mDisplacements;
}

Vectors& FEMObject::getElasticForces()
{
    return mForcesEl;
}

Vectors& FEMObject::getExternalForces()
{
    return mForcesExt;
}

Vector& FEMObject::getExternalForce(size_t id)
{
    return mForcesExt[id];
}

std::shared_ptr<Polygon3D> FEMObject::getPolygon()
{
    return mPoly3;
}

const Eigen::SparseMatrix<double>& FEMObject::getStiffnessMatrix(bool corot)
{
    if (corot)
        return mKCorot.getMatrix();
    else
        return mK.getMatrix();
}

Vector& FEMObject::getPosition(size_t id)
{
    return mPositions[id];
}

void FEMObject::setPosition(Vector v, ID id)
{
    mPositions[id] = v;
}

void FEMObject::addToPosition(Vector v, ID id)
{
    mPositions[id] += v;
}

size_t FEMObject::getSize()
{
    return mPositions.size();
}

GeometricData* FEMObject::getGeometricData()
{
    return mPoly3.get();
}

void FEMObject::initializeStiffnessMatrix()
{
    for (FiniteElement& fe : mFiniteElements)
    {
        std::array<unsigned int, 4> cell = fe.getCell();
        for (unsigned int a = 0; a < 4; ++a)
        {
            Eigen::Index rSub = cell[a];
            for (unsigned int b = 0; b < 4; ++b)
            {
                Eigen::Index cSub = cell[b];
                mKCorot.addSubMatrix(rSub, cSub);
                mK.addSubMatrix(rSub, cSub);
            }
        }
    }

    mK.assemble();
    mKCorot.assemble();

    // initialize mKColPtrs and mKCorotColPtrs
    mKColPtrs.clear();
    mKCorotColPtrs.clear();

    mKColPtrs.reserve(mFiniteElements.size());
    mKCorotColPtrs.reserve(mFiniteElements.size());

    for (FiniteElement& fe : mFiniteElements)
    {
        FEColumnPtrs ptrs;
        FEColumnPtrs ptrsCorot;
        std::array<unsigned int, 4> cell = fe.getCell();
        for (unsigned int a = 0; a < 4; ++a)
        {
            Eigen::Index r_major = cell[a];
            for (unsigned int b = 0; b < 4; ++b)
            {
                Eigen::Index c_major = cell[b];
                ptrs[a][b] = mK.createColumnPtrs(r_major, c_major);
                ptrsCorot[a][b] = mKCorot.createColumnPtrs(r_major, c_major);
            }
        }
        mKColPtrs.push_back(ptrs);
        mKCorotColPtrs.push_back(ptrsCorot);
    }

//    updateStiffnessMatrix(false);
    updateFEM(false);

    mAnalyzePatternNecessary = true;
}

void FEMObject::updateElasticForces()
{
    START_TIMING_SIMULATION("FEMObject::updateElasticForces:1")
    for (FiniteElement& fe : mFiniteElements)
        fe.updateCorotatedForces();
    STOP_TIMING_SIMULATION;
    START_TIMING_SIMULATION("FEMObject::updateElasticForces:2")
    assembleElasticForces();
    STOP_TIMING_SIMULATION;
}

void FEMObject::updateStiffnessMatrix(bool corotated)
{
    START_TIMING_SIMULATION("FEMObject::updateStiffnessMatrix()");
    for (FiniteElement& fe : mFiniteElements)
    {
        if (corotated)
            fe.updateRotation();
        fe.updateStiffnessMatrix(corotated);
    }
    STOP_TIMING_SIMULATION;

    START_TIMING_SIMULATION("FEMObject::assembleStiffnessMatrix()");
    assembleStiffnessMatrix(corotated);
    STOP_TIMING_SIMULATION;
}

void FEMObject::updateMassMatrix()
{
    mMassCoef.clear();
    mMassCoef.reserve(mFiniteElements.size() * 12);
    for (FiniteElement& fe : mFiniteElements)
    {
        for (unsigned int a = 0; a < 4; ++a)
        {
            unsigned int a_global = fe.getCell()[a];
            double value = fe.getM()[a];
            for (unsigned int i = 0; i < 3; ++i)
            {
                unsigned int r = a_global * 3 + i;
                unsigned int c = a_global * 3 + i;
                mMassCoef.push_back(Triplet<double>(r, c, value));
            }

        }
    }
    // update sparse matrix
    //std::cout << mM << "\n";
    mM.setZero();
    mM.setFromTriplets(mMassCoef.begin(), mMassCoef.end());
}

void FEMObject::assembleElasticForces()
{
    unsigned int size = static_cast<unsigned int>(mForcesEl.size());
    VectorXd::Map(mForcesEl[0].data(), size * 3)
            = VectorXd::Zero(size * 3);

    for (FiniteElement& fe : mFiniteElements)
    {
        std::array<unsigned int, 4> cell = fe.getCell();
        for (size_t a = 0; a < 4; ++a)
        {
            mForcesEl[cell[a]] += fe.getElasticForces()[a];
        }
    }
}

void FEMObject::assembleStiffnessMatrix(bool corotated)
{
    // upper symmetric matrix: currently, not in use

    // fill tripplet
    // there are per finite element 4*4*3*3=144 entries
    // this is a 12x12 matrix with the elements at
    // r = a * 3 + i
    // c = b * 3 + k

    if (corotated)
        mKCorot.setZero();
    else
        mK.setZero();

    for (size_t i = 0; i < mFiniteElements.size(); ++i)
    {
        FiniteElement& fe = mFiniteElements[i];
        const FEColumnPtrs& ptrs = corotated ? mKCorotColPtrs[i] : mKColPtrs[i];

        for (unsigned int a = 0; a < 4; ++a)
        {
            for (unsigned int b = 0; b < 4; ++b)
            {
                if (corotated)
                {
                    Eigen::Map<Eigen::Vector3d>(ptrs[a][b][0]) +=
                            fe.getKCorot()[a][b].col(0);
                    Eigen::Map<Eigen::Vector3d>(ptrs[a][b][1]) +=
                            fe.getKCorot()[a][b].col(1);
                    Eigen::Map<Eigen::Vector3d>(ptrs[a][b][2]) +=
                            fe.getKCorot()[a][b].col(2);
                }
                else
                {
                    Eigen::Map<Eigen::Vector3d>(ptrs[a][b][0]) +=
                            fe.getK()[a][b].col(0);
                    Eigen::Map<Eigen::Vector3d>(ptrs[a][b][1]) +=
                            fe.getK()[a][b].col(1);
                    Eigen::Map<Eigen::Vector3d>(ptrs[a][b][2]) +=
                            fe.getK()[a][b].col(2);
                }

            }
        }
    }
}

bool FEMObject::factorize(const Eigen::SparseMatrix<double>& A)
{
#if 1
        // if solver is sparse LU
//        solver.analyzePattern(A);
//        if (!solver.lastErrorMessage().empty())
//            printf("EigenErrorMessage[analyzePattern]: %s\n", solver.lastErrorMessage().c_str());
//        solver.factorize(A);
//        if (!solver.lastErrorMessage().empty())
//            fprintf(stderr, "EigenErrorMessage[factorize]: %s\n", solver.lastErrorMessage().c_str());

        if (mAnalyzePatternNecessary)
        {
            START_TIMING_SIMULATION("FEMObject::solveFEM()::linSolver::analyze");
            mSolver.analyzePattern(A);
            mAnalyzePatternNecessary = false;
            STOP_TIMING_SIMULATION;
        }

        if (mSolver.info() != Eigen::Success)
        {
            std::cerr << "Solver: analyzePattern failed.\n";
            return false;
        }

        START_TIMING_SIMULATION("FEMObject::solveFEM()::linSolver::factorize");
        mSolver.factorize(A);
        STOP_TIMING_SIMULATION;

#else

        START_TIMING_SIMULATION("FEMObject::solveFEM()::linSolver::factorize");
        mSolver.compute(A);
        STOP_TIMING_SIMULATION;

#endif
    return true;
}
