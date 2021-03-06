#ifndef FINITEELEMENT_H
#define FINITEELEMENT_H

#include "FEMObject.h"
#include "data_structures/DataStructures.h"
#include "simulation/ElasticMaterial.h"
#include <array>

// Wrapper for cells
class FiniteElement
{
public:
    FiniteElement(FEMObject* so, Cell cell, ElasticMaterial material);

    // Forces an update of the stiffness matrix if Linear FEM (not corotated)
    // is used for simulation.
    void setMaterial(ElasticMaterial material);

    // Getters
    std::array<Eigen::Vector, 4>& getElasticForces()
    {
        return mForces;
    }

    std::array<unsigned int, 4>& getCell()
    {
        return mCell;
    }

    const std::array<std::array<Eigen::Matrix3d, 4>, 4>& getK()
    {
        return mK;
    }

    const std::array<std::array<Eigen::Matrix3d, 4>, 4>& getKCorot()
    {
        return mKCorot;
    }

    const std::array<double, 4>& getM()
    {
        return mM;
    }

    void setM(const std::array<double, 4>& mass)
    {
        mM = mass;
    }

    double getVolume() const
    {
        return mVolume;
    }

    ElasticMaterial& getMaterial()
    {
        return mMaterial;
    }

    // Updates the stiffness matrix w.r.t. the initial configuration.
    void initialize();

    // Updates the current rotation of this finite element by applying
    // a singular value decomposition (SVD).
    void updateRotation();

    // Update rotation according to
    // "A Robust Method to Extract the Rotational Part of Deformations".
    // This is an iterative approach which avoids the useage of a costly
    // singular value decomposition. Ideally, only a few iterations are needed.
    // This is especially true if the object doesn't rotate too much inbetween
    // time steps.
    void updateRotationFast(size_t maxIterations = 20, double tolerance = 1e-6);

    // Updates stiffness matrix and forces.
    void update(bool corotated);

    // update stiffness matrix
        void updateStiffnessMatrix(bool corotated);

        // Updates the stiffness matrix according to the linear FEM.
        void updateLinearStiffnessMatrix();

        // Calculates the corotated stiffness matrix and stores it in mKCorot.
        // Requires a call of updateK() on the initial configuration.
        // If positions changed, requires a call to updateRotation().
        void updateCorotatedStiffnessMatrix();

        // Updates plasticity matrix According to equation 15 in
        // "Interactive Virtual Materials" by Mueller and Gross.
        // Only needs to be executed once in the initialization.
        void updatePlasticityMatrix();

    // Update forces
        // Calculates and updates the forces according to the FEM.
        // \param corotated - if true, calculates forces according to corotated FEM,
        //      if false, according to linear FEM.
        void updateForces(bool corotated);

        // Calculates and updates mForces accordign to the corotated FEM:
        // F += R * K[a][b] * (R.transpose() * y(b) - x(b));
        void updateCorotatedForces();

        // Calculates and updates mForces according to the linear FEM:
        // F += K[a][b] * u(b);
        void updateLinearForces();

        void updatePlasticForces(double stepSize);

private:

    // calculates the deformation gradient F
    void updateF();

    // calculates cauchy strain and cauchy stress of linear material law
    void updateCauchyStressStrain();

    // update the derivative of the deformation gradient for the initial position
    void updateDnx();

    // update the derivative of the deformation gradient for the current position
    void updateDny();

    // deformation u = y - x
    const Eigen::Vector& u(size_t i)
    {
        return mFemObj->getDisplacements()[mCell[i]];
    }
    // initial position x
    const Eigen::Vector& x(size_t i)
    {
        return mFemObj->getInitialPositions()[mCell[i]];
    }
    // current position y
    const Eigen::Vector& y(size_t i)
    {
        return mFemObj->getPositions()[mCell[i]];
    }
    // velocity v
    const Eigen::Vector& v(size_t i)
    {
        return mFemObj->getVelocities()[mCell[i]];
    }
    // forces f
    const Eigen::Vector& f(size_t i)
    {
        return mFemObj->getElasticForces()[mCell[i]];
    }

    FEMObject* mFemObj;
    std::array<unsigned int, 4> mCell;
    ElasticMaterial mMaterial;

    Eigen::Matrix3d mR;
    Eigen::Quaterniond mQ; // Used for fast rotation calculation.
    Eigen::Matrix3d mF; // Deformation tensor
    Eigen::Matrix<double, 6, 12> mB; // Deformation tensor rewritten.
    Eigen::Matrix3d mCauchyStrain;
    Eigen::Matrix3d mCauchyStress;
    std::array<Eigen::Vector, 4> mDnx;
    std::array<Eigen::Vector, 4> mDny;
    std::array<Eigen::Vector ,4> mForces;
    std::array<std::array<Eigen::Matrix3d, 4>, 4> mK;
    std::array<std::array<Eigen::Matrix3d, 4>, 4> mKCorot;
    std::array<double, 4> mM;
    double mDensity;
    double mVolume;

    Eigen::Matrix<double, 12, 6> mP; // Plsticity matrix
    // The current plastic strain that causes a non-elastic deformation.
    Eigen::Matrix<double, 6, 1> mStrainPlastic;

    // This flag is only relevant for the Linear FEM simulation (not corotated).
    // Is true if the stiffnessmatrix requires an update.
    bool mStiffnessMatrixDirty;
};

#endif // FINITEELEMENT_H
