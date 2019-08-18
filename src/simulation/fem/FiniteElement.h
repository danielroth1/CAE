#ifndef FINITEELEMENT_H
#define FINITEELEMENT_H

#include "data_structures/DataStructures.h"
#include "simulation/ElasticMaterial.h"
#include <array>

class FEMObject;

// Wrapper for cells
class FiniteElement
{
public:
    FiniteElement(FEMObject* so, Cell cell, ElasticMaterial material);

    // Forces an update of the stiffness matrix if Linear FEM (not corotated)
    // is used for simulation.
    void setMaterial(ElasticMaterial material);

    // Getters
    std::array<Eigen::Vector, 4>& getElasticForces();
    std::array<unsigned int, 4>& getCell();
    const std::array<std::array<Eigen::Matrix3d, 4>, 4>& getK();
    const std::array<std::array<Eigen::Matrix3d, 4>, 4>& getKCorot();
    const std::array<double, 4>& getM();
    ElasticMaterial& getMaterial();

    // Updates the stiffness matrix w.r.t. the initial configuration.
    void initialize();

    // Updates the current rotation of this finite element by applying
    // a singular value decomposition (SVD).
    void updateRotation();

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
    const Eigen::Vector& u(size_t i);
    // initial position x
    const Eigen::Vector& x(size_t i);
    // current position y
    const Eigen::Vector& y(size_t i);
    // velocity v
    const Eigen::Vector& v(size_t i);
    // forces f
    const Eigen::Vector& f(size_t i);

    FEMObject* mFemObj;
    std::array<unsigned int, 4> mCell;
    ElasticMaterial mMaterial;

    Eigen::Matrix3d mR;
    Eigen::Matrix3d mF;
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

    // This flag is only relevant for the Linear FEM simulation (not corotated).
    // Is true if the stiffnessmatrix requires an update.
    bool mStiffnessMatrixDirty;
};

#endif // FINITEELEMENT_H
