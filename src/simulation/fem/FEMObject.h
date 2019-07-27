#ifndef FEMOBJECT_H
#define FEMOBJECT_H



#include <data_structures/DataStructures.h>
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
class FEMObject : public SimulationObject
{
public:

    // Initial positions are equal to given positions.
    FEMObject(
            Domain* domain,
            std::shared_ptr<Polygon3D> poly3);

    virtual ~FEMObject() override;

    virtual SimulationObject::Type getType() const override;

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

    Truncation* getTruncation();

    std::shared_ptr<Polygon3D> getPolygon();

    // SimulationObject interface
public:
    virtual Eigen::Vector& getPosition(size_t id) override;
    virtual void setPosition(Eigen::Vector v, ID id) override;
    virtual void addToPosition(Eigen::Vector v, ID id) override;
    virtual size_t getSize() override;
    virtual GeometricData* getGeometricData() override;

private:

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

    ID mId;

    // Polygon3D
    // Is used to inform other modules about updates in the data.
    std::shared_ptr<Polygon3D> mPoly3;

    // The linear solver
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>, Eigen::Lower> mSolver;

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

    std::vector<Eigen::Triplet<double>> mCoefficients;
    Eigen::SparseMatrix<double> mK;
    Eigen::SparseMatrix<double> mKCorot;
    std::vector<Eigen::Triplet<double>> mMassCoef;
    Eigen::SparseMatrix<double> mM;

    std::shared_ptr<Truncation> mTruncation;

};

#endif // FEMOBJECT_H
