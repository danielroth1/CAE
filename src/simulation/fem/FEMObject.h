#ifndef FEMOBJECT_H
#define FEMOBJECT_H



#include <data_structures/DataStructures.h>
#include <data_structures/SparseMatrix33.h>
#include "FiniteElement.h"
#include <Eigen/Sparse>
#include <simulation/SimulationObject.h>

class Polygon3D;
class Truncation;

// A FEMObject describes a deformable body and offers
// functionality to deform that body with the help of
// the FEM. The geometry must be discretized in finite
// elements (in this case tetrahedrons), i.e. the geometry
// is a Polygon3D.
//
// This class offers the functionality to simulate the corotated
// FEM implicitly. To do so, do the following:
// Init:
//      -> init the stiffness matrix with "initializeFEM()"
// Simulation loop:
//      1. update the stiffness matrix and forces with updateCorotatedFEM()
//      2. apply the solver with
//
// Optimzations:
// -> analyzePattern() is only called once when the sparsity pattern of the
//      linear systems matrix changes. As long as the sparsity pattern doesn't
//      change, a recomputation is unnecessary, see " The Compute Step" under
//      https://eigen.tuxfamily.org/dox/group__TopicSparseSystems.html
//      The impact of this optimzation depends on the matrix structure. It can
//      make up from 5% to 50% of the total compute time (i.e. analyzePattern()
//      + factorize() call).
//
// -> usage of a sparse matrix that is especially suited for the FEM, see
//      SparseMatrix33.
//
class FEMObject : public SimulationObject
{
public:

    // Initial positions are equal to given positions.
    FEMObject(
            Domain* domain,
            std::shared_ptr<Polygon3D> poly3);

    virtual ~FEMObject() override;

    virtual void accept(SimulationObjectVisitor& visitor) override;

    virtual void applyImpulse(
            SimulationPointRef& ref, const Eigen::Vector& impulse) override;

    virtual void applyForce(
            SimulationPointRef& ref, const Eigen::Vector& force) override;

    // Getters
    ID getId() const;

    // Setters
    void setId(ID id);

    // Update Functions

    // calculates all dnx and K matrices
    // call this method at the start of the simulation
    // it initializes the stiffness matrix for the implicit
    // linera finite element method
    void initializeFEM();

    // update the displacements if necessary
    void updateDisplacements();

    // update the positions
    // only call this method if the displacements are given
    void updatePositions();

    // call this method in the simulation loop
    void updateFEM(bool corotated);

    // TODO: comment this method
    void solveFEMExplicitly(double timeStep, bool corotated);

    // calls a linear solver
    // updates m_delta_v
    // (M + h*h*K) \delta v = h * (f - h * K * v_0)
    // requires the calculation of
    // - mass matrix
    // - forces
    // - stiffness matrix
    // => call updateCorotatedFEM()
    // \param firstStep - if true, the whole solver step is computed
    //      if false, the matrix assembling and linear solver factorization
    //      from the last time when this methos was called with firstStep = true
    //      is reused. Typically, this kind of iterations are basically free since
    //      they are hundert times faster.
    void solveFEM(double stepSize, bool corotated, bool firstStep = true);

    // Performs:
    // v = v - \delta v_{i+1}
    void revertSolverStep();

    virtual void integratePositions(double stepSize) override;

    virtual void revertPositions() override;

    void updateGeometricData();

    void applyImpulse(ID vertexIndex, const Eigen::Vector& impulse);
    void applyForce(ID vertexIndex, const Eigen::Vector& force);

    void addTrunctionIds(const std::vector<ID>& vectorIDs);

    void removeTrunctionIds(const std::vector<ID>& vectorIDs);

    void clearTruncation();

    const std::vector<ID>& getTruncatedVectorIds() const;

    Eigen::Vector& x(unsigned int i);
    Eigen::Vector& y(unsigned int i);
    Eigen::Vector& u(unsigned int i);
    Eigen::Vector& v(unsigned int i);
    Eigen::Vector& f_el(unsigned int i);

    void setYoungsModulus(double youngsModulus);
    double getYoungsModulus();

    void setPoissonRatio(double poissonRatio);
    double getPoissonRatio();

    void setElasticMaterial(const ElasticMaterial& material);
    ElasticMaterial getElasticMaterial();

    // Getters
    Vectors& getInitialPositions();
    Vectors& getPositions();
    const Eigen::Vector& getPositionPrevious(size_t index) const;
    Vectors& getVelocities();
    double getMass(ID vertexId);

    Vectors& getDisplacements();
    Vectors& getElasticForces();
    Vectors& getExternalForces();
    Eigen::Vector& getExternalForce(size_t id);
    //std::vector<FiniteElement>& getFiniteElements() { return m_finite_elements; }

    std::shared_ptr<Polygon3D> getPolygon();

    const Eigen::SparseMatrix<double>& getStiffnessMatrix(bool corot);

    // SimulationObject interface
public:
    virtual Eigen::Vector& getPosition(size_t id) override;
    virtual void setPosition(Eigen::Vector v, ID id) override;
    virtual void addToPosition(Eigen::Vector v, ID id) override;
    virtual size_t getSize() override;
    virtual GeometricData* getGeometricData() override;

private:

    typedef std::array<std::array<std::array<double*, 3>, 4>, 4> FEColumnPtrs;

    void initializeStiffnessMatrix();

    // TOOD: assembling and calculating can be combined?

    // Assemles the global forces from each finite elements and writes them
    // in mForcesEl.
    void updateElasticForces();

    // Assembles and updtes the global stiffness matrix mK or mKCorot from
    // the local stiffness matrices of the FiniteElements.
    // Updates mK if corotated is false or mKCorot if corotated is true.
    void updateStiffnessMatrix(bool corotated);

    // Assambles the global mass matrix from the local masses of each finite
    // element.
    void updateMassMatrix();

    void assembleElasticForces();
    void assembleStiffnessMatrix(bool corotated);

    // Factorizes the given matrix.
    // \return true if this was succesfull. It can be unsuccesfull if the
    //  matrix is singular.
    bool factorize(const Eigen::SparseMatrix<double>& A);

    ID mId;

    // Polygon3D
    // Is used to inform other modules about updates in the data.
    std::shared_ptr<Polygon3D> mPoly3;

    // The linear solver
    Eigen::SimplicialLLT<Eigen::SparseMatrix<double>> mSolver;

    // Geometric Data references
    Vectors mPositions;
    Vectors mPositionsPrevious;

    // Permanent simulation data
    Vectors mInitialPositions;

    // Current velocities of this object. This is different from
    // mVelocitiesFEM when there other forces/impulses applied
    // after the solveFEM call: v_{i+1}
    Vectors mVelocities;

    // The velocities from the previous time step: v_i
    Vectors mVelocitiesPrevious;
    std::vector<double> mMasses;


    // Temporary data for simulation calculation
    Vectors mDisplacements;
    Vectors mDisplacementsPrevious;
    Vectors mForcesEl;
    Vectors mForcesExt;
    Vectors mDeltaV;
    std::vector<FiniteElement> mFiniteElements;

    // Contains the linear part of the global system matrix. If corotated is
    // enabled, is used to update mKCorot.
    SparseMatrix33 mK;
    std::vector<FEColumnPtrs> mKColPtrs;

    // Contains the global system matrix if corotated fem is enabled. Is updated
    // in each time step.
    SparseMatrix33 mKCorot;
    std::vector<FEColumnPtrs> mKCorotColPtrs;

    std::vector<Eigen::Triplet<double>> mMassCoef;
    Eigen::SparseMatrix<double> mM;

    std::shared_ptr<Truncation> mTruncation;

    // If true, triggers a recomputation of mSolver->analyzePattern() in the
    // next solver step. It's then automatically set to false again.
    bool mAnalyzePatternNecessary;

};

#endif // FEMOBJECT_H
