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

// FEMObject without initialPositions

using namespace Eigen;

FEMObject::FEMObject(
        Domain* domain,
        std::shared_ptr<Polygon3D> poly3)
    : SimulationObject(domain)
    , mPoly3(poly3)
    , mPositions(poly3->getPositions())
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
    double E = 5e+4;
    ElasticMaterial material;
    material.setFromYoungsPoisson(E, v);

    const Cells& cells = poly3->getTopology3D().getCells();
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
    mK = SparseMatrix<double>(nVertices*3, nVertices*3);
    mKCorot = SparseMatrix<double>(nVertices*3, nVertices*3);
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

SimulationObject::Type FEMObject::getType() const
{
    return SimulationObject::Type::FEM_OBJECT;
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
    updateStiffnessMatrix(false);
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
    for (FiniteElement& fe : mFiniteElements)
        fe.update(corotated);
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

Truncation* FEMObject::getTruncation()
{
    return mTruncation.get();
}

std::shared_ptr<Polygon3D> FEMObject::getPolygon()
{
    return mPoly3;
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

    //std::cout << "u: " << VectorXd::Map(mDisplacements[0].data(), size * 3).norm() << "\n";

}

void FEMObject::solveFEM(double timeStep, bool corotated, bool firstStep)
{
    if (mTruncation->getTruncatedVectorIds().size() == mPositions.size())
        return;

    if (!firstStep)
    {
        for (FiniteElement& fe : mFiniteElements)
            fe.updateCorotatedForces();
        updateElasticForces();
    }

    // map velocities to eigen struct
    unsigned int size = static_cast<unsigned int>(mVelocities.size());

    Eigen::Map<VectorXd> v = VectorXd::Map(mVelocities[0].data(), size * 3);

    VectorXd f = VectorXd::Map(mForcesExt[0].data(), size * 3)
            - VectorXd::Map(mForcesEl[0].data(), size * 3);

    SparseMatrix<double>& K = mKCorot;
    if (!corotated)
        K = mK;
    SparseMatrix<double> A = mM + timeStep * timeStep * K;
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

    // good truncation
    //Truncation truncation(A, b, mTruncatedVectorIds);

    SparseMatrix<double> ACopy = A;
    VectorXd bCopy = b;
    mTruncation->truncateByRemoving(ACopy, bCopy, A, b);

    SparseLU<SparseMatrix<double>> solver;
    VectorXd sol(size*3);
    solver.analyzePattern(A);
    if (!solver.lastErrorMessage().empty())
        printf("EigenErrorMessage[analyzePattern]: %s\n", solver.lastErrorMessage().c_str());
    solver.factorize(A);
    if (!solver.lastErrorMessage().empty())
        fprintf(stderr, "EigenErrorMessage[factorize]: %s\n", solver.lastErrorMessage().c_str());
    sol = solver.solve(b);
    sol = mTruncation->createOriginal(sol);

    // update the velocities
    VectorXd::Map(mDeltaV[0].data(), size * 3) = sol;

    VectorXd::Map(mVelocities[0].data(), size * 3) +=
            VectorXd::Map(mDeltaV[0].data(), size * 3);

    integratePositions(timeStep);

//    std::cout << "norm = " << sol.norm() << "\n";

    // truncation test
    // truncation TODO: not working because of zero columns
//    SparseMatrix<double> A_trunc = A;
//    VectorXd b_trunc = b;
//    Truncation truncation(A_trunc, b_trunc, mTruncatedVectorIds);
//    A_trunc = truncation.getTruncatedA();
//    b_trunc = truncation.getTruncatedB();
//    SparseLU<SparseMatrix<double>> solver_trunc;
//    VectorXd sol_trunc(size*3);
//    solver_trunc.analyzePattern(A_trunc);
//    solver_trunc.factorize(A_trunc);
//    sol_trunc = solver_trunc.solve(b_trunc);
//    std::cout << "A_trunc:\n" << A_trunc << "\n\n";
//    std::cout << "b_trunc:\n" << b_trunc << "\n\n";
//    std::cout << "sol_trunc:\n" << sol_trunc << "\n\n";
//    sol_trunc = truncation.createOriginal(sol_trunc);

    // bad truncation

//    for (unsigned int id : mTruncatedVectorIds)
//    {
//        for (unsigned int r = 0; r < 3; ++r)
//        {
//            unsigned int r_global = id*3+r;
//            // truncation of b
//            b(r_global) = 0.0;

//            // truncation of A
//            for (unsigned int c = 0; c < mPositions.size()*3; ++c)
//            {
//                unsigned int c_global = c;
//                // TODO: inefficient but sufficient for now
//                if (r_global == c_global)
//                    A.coeffRef(r_global, c_global) = 1.0;
//                else
//                {
//                    A.coeffRef(r_global, c_global) = 0.0;
//                    A.coeffRef(c_global, r_global) = 0.0;
//                }
//            }

//        }
//    }

//    VectorXd sol(size*3);
//    SparseLU<SparseMatrix<double>> solver;
//    solver.analyzePattern(A);
//    if (!solver.lastErrorMessage().empty())
//        printf("EigenErrorMessage[analyzePattern]: %s\n", solver.lastErrorMessage().c_str());
//    solver.factorize(A);
//    if (!solver.lastErrorMessage().empty())
//        fprintf(stderr, "EigenErrorMessage[factorize]: %s\n", solver.lastErrorMessage().c_str());

//    sol = solver.solve(b);

//    double res = (A*sol).norm();
//    std::cout << "resitual after solve: " << res << "\n";



//    std::cout << "sol before:\n" << sol << "\n";
//    sol = truncation.createOriginal(sol);
//    std::cout << "sol after:\n" << sol << "\n";
    //    solver.compute(A);
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
    // This call is necessary to inform the other modules about
    // the change in position of the underlying Polygon3D that is
    // simulated here.
    mPoly3->geometricDataChanged();
}

void FEMObject::applyImpulse(ID vertexIndex, const Vector& impulse)
{
    // TODO: error somewhere here, NAN
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

void FEMObject::updateElasticForces()
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

void FEMObject::initializeStiffnessMatrix()
{
    updateStiffnessMatrix(false);
}

void FEMObject::updateStiffnessMatrix(bool corotated)
{
    // fill tripplet
    // there are per finite element 4*4*3*3=144 entries
    // this is a 12x12 matrix with the elements at
    // r = a * 3 + i
    // c = b * 3 + k
    mCoefficients.clear();
    mCoefficients.reserve(mFiniteElements.size() * 144);
    for (FiniteElement& fe : mFiniteElements)
    {
        std::array<unsigned int, 4> cell = fe.getCell();
        for (unsigned int a = 0; a < 4; ++a)
        {
            for (unsigned int b = 0; b < 4; ++b)
            {
                for (unsigned int i = 0; i < 3; ++i)
                {
                    for (unsigned int k = 0; k < 3; ++k)
                    {
                        size_t r = cell[a] * 3 + i;
                        size_t c = cell[b] * 3 + k;
                        double value;
                        if (corotated)
                            value = fe.getKCorot()[a][b](i,k);
                        else
                            value = fe.getK()[a][b](i,k);

                        // TODO: pay attention to when
                        // a, cell[a], a * 3, cell[a] * 3 is called
                        // -> * 3 is used if the structure saved double values
                        // it is not used if it uses Matrix3d or Vector3d
                        // -> a is used if the structure saves local values
                        // cell[a] is used if the structure saved global values
                        // a*3+i => FEM with double array
                        // a => FEM with Matrix3d or Vector3d
                        // cell[a] * 3 + i => global with double array
                        // cell[a] => global with Matrix3d or Vector3d

                        mCoefficients.push_back(Triplet<double>(r, c, value));
                    }
                }
            }
        }
    }

    // update sparse matrix
    if (corotated)
    {
        mKCorot.setZero();
        mKCorot.setFromTriplets(mCoefficients.begin(), mCoefficients.end());
    }
    else
    {
        mK.setZero();
        mK.setFromTriplets(mCoefficients.begin(), mCoefficients.end());
    }
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

Vectors& FEMObject::getPositions()
{
    return mPositions;
}

Vectors& FEMObject::getDisplacements()
{
    return mDisplacements;
}

Vectors& FEMObject::getInitialPositions()
{
    return mInitialPositions;
}

Vectors& FEMObject::getVelocities()
{
    return mVelocities;
}

double FEMObject::getMass(ID vertexId)
{
    return mMasses[vertexId];
}

const Vector& FEMObject::getPositionPrevious(size_t index) const
{
    return mPositionsPrevious[index];
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
