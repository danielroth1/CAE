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

    // Getters
    std::array<Eigen::Vector, 4>& getElasticForces();
    std::array<unsigned int, 4>& getCell();
    std::array<std::array<Eigen::Matrix3d, 4>, 4> getK();
    std::array<std::array<Eigen::Matrix3d, 4>, 4> getKCorot();
    std::array<double, 4> getM();

    void initialize();

    void update(bool corotated);

    // calculates the deformation gradient F
    void updateF();

    // calculates cauchy strain and cauchy stress of linear material law
    void updateCauchyStressStrain();

    // update the derivative of the deformation gradient for the initial position
    void updateDnx();

    // update the derivative of the deformation gradient for the current position
    void updateDny();

    // update the stiffness matrix for this finite element
    // A linear matrial law is assumed
    void updateK();

    void updateCorotatedK();

    void updateForces();

    void updateCorotatedForces();

    void updateRotation();

    void updateGlobalValues();


private:

    // deformation u = y - x
    Eigen::Vector u(size_t i);
    // initial position x
    Eigen::Vector x(size_t i);
    // current position y
    Eigen::Vector y(size_t i);
    // velocity v
    Eigen::Vector v(size_t i);
    // forces f
    Eigen::Vector f(size_t i);

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
};

#endif // FINITEELEMENT_H
