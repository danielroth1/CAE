#include "FiniteElement.h"

#include "FEMObject.h"
#include <Eigen/Core>
#include <iostream>

using namespace Eigen;

FiniteElement::FiniteElement(
        FEMObject* femObj,
        Cell cell,
        ElasticMaterial material)
    : mFemObj(femObj)
    , mMaterial(material)
{
    mDensity = 100.0;
    mCell = cell;
    mStiffnessMatrixDirty = false;
}

void FiniteElement::setMaterial(ElasticMaterial material)
{
    mMaterial = material;
    mStiffnessMatrixDirty = true;
}

std::array<Vector, 4>& FiniteElement::getElasticForces()
{
    return mForces;
}

std::array<unsigned int, 4>& FiniteElement::getCell()
{
    return mCell;
}

std::array<std::array<Matrix3d, 4>, 4> FiniteElement::getK()
{
    return mK;
}

std::array<std::array<Matrix3d, 4>, 4> FiniteElement::getKCorot()
{
    return mKCorot;
}

std::array<double, 4> FiniteElement::getM()
{
    return mM;
}

ElasticMaterial& FiniteElement::getMaterial()
{
    return mMaterial;
}

Vector FiniteElement::u(size_t i)
{
    return mFemObj->getDisplacements()[mCell[i]];
}

Vector FiniteElement::x(size_t i)
{
    return mFemObj->getInitialPositions()[mCell[i]];
}

Vector FiniteElement::y(size_t i)
{
    return mFemObj->getPositions()[mCell[i]];
}

Vector FiniteElement::v(size_t i)
{
    return mFemObj->getVelocities()[mCell[i]];
}

Vector FiniteElement::f(size_t i)
{
    return mFemObj->getElasticForces()[mCell[i]];
}

void FiniteElement::initialize()
{
    updateDnx();
    updateF();
    updateCauchyStressStrain();
    updateLinearStiffnessMatrix();
}

void FiniteElement::updateRotation()
{
    updateF();
    JacobiSVD<Matrix3d> svd(mF, ComputeFullU | ComputeFullV);
    double det = (svd.matrixU() * svd.matrixV().transpose()).determinant();
    Matrix3d C = Matrix3d::Identity();
    C(2,2) = det;
    mR = svd.matrixU() * C * svd.matrixV().transpose();
//    if (det < 0)
//        std::cout << "inversion detected! det = " << det << "\n";

}

void FiniteElement::update(bool corotated)
{
    if (corotated)
    {
        updateCorotatedStiffnessMatrix();
        updateCorotatedForces();
    }
    else
    {
        updateLinearForces();
    }
}

void FiniteElement::updateStiffnessMatrix(bool corotated)
{
    if (corotated)
        updateCorotatedStiffnessMatrix();
    else if (mStiffnessMatrixDirty)
        updateLinearStiffnessMatrix();
}

void FiniteElement::updateLinearStiffnessMatrix()
{
    for (size_t a = 0; a < 4; ++a)
    {
        for (size_t b = 0; b < 4; ++b)
        {
            double second_part = mMaterial.getLameMu() * mDnx[a].dot(mDnx[b]);
            Matrix3d& K = mK[a][b];
            for (Index i = 0; i < 3; ++i)
            {
                for (Index k = 0; k < 3; ++k)
                {
                    //K(i,k) = ...
                    double value = 0.0;
                    if (i == k)
                        value = second_part;
                    value += mMaterial.getLameLambda() * mDnx[a](i) * mDnx[b](k)
                            + mMaterial.getLameMu() * mDnx[a](k) * mDnx[b](i);
                    K(i,k) = value * mVolume;
                }
            }
        }
        double mass = (mDensity * mVolume) / 4;
        mM[a] = mass;// * Matrix3d::Identity();
    }
}

void FiniteElement::updateCorotatedStiffnessMatrix()
{
    for (size_t a = 0; a < 4; ++a)
    {
        for (size_t b = 0; b < 4; ++b)
        {
            Matrix3d& K = mK[a][b];
            Matrix3d& K_corot = mKCorot[a][b];
            K_corot = mR * K * mR.transpose();
        }
    }
}

void FiniteElement::updateForces(bool corotated)
{
    if (corotated)
        updateCorotatedForces();
    else
        updateLinearForces();
}

void FiniteElement::updateCorotatedForces()
{
//    updateRotation(); // TODO: if not already done
    for (size_t a = 0; a < 4; ++a)
    {
        mForces[a] = Vector::Zero();
        for (size_t b = 0; b < 4; ++b)
        {
            mForces[a] += mR * mK[a][b] *
                    (mR.transpose() * y(b) - x(b));
//            f(a) += mR * mK[a][b] *
//                    (mR.transpose() * y(b) - x(b));
        }
    }
}

void FiniteElement::updateLinearForces()
{
    for (size_t a = 0; a < 4; ++a)
    {
        mForces[a] = Vector::Zero();
        for (size_t b = 0; b < 4; ++b)
        {
            mForces[a] += mK[a][b] * u(b);

            // directly globally updates them this way
            // but this is not wanted here
            //f(a) += mK[a][b] * u(b);
        }
    }
}

void FiniteElement::updateGlobalValues()
{
    for (size_t a = 0; a < 4; ++a)
    {
        f(a) += mForces[a];
        for (size_t b = 0; b < 4; ++b)
        {

        }
    }
}

void FiniteElement::updateF()
{
//    mF = Matrix3d::Identity();
//    for (size_t i = 0; i < 4; ++i) {
//        mF += u(i) * mDnx[i].transpose();
//    }
    mF = Matrix3d::Zero();
    for (size_t i = 0; i < 4; ++i) {
        mF += y(i) * mDnx[i].transpose();
    }
}

void FiniteElement::updateCauchyStressStrain()
{
    // Cauchy strain
    mCauchyStrain = 0.5 * (mF + mF.transpose()) - Matrix3d::Identity();
    double trace = mCauchyStrain.trace();
    for (Index i = 0; i < 3; ++i)
         for (Index j = 0; j < 3; ++j)
            mCauchyStress(i, j) = (i == j) * mMaterial.getLameLambda() * trace
                    + 2 * mMaterial.getLameMu() * mCauchyStrain(i, j);
}

void FiniteElement::updateDnx()
{
    // As described in
    // http://www.iue.tuwien.ac.at/phd/nentchev/node30.html and
    // http://www.iue.tuwien.ac.at/phd/nentchev/node31.html

    Vector r1 = mFemObj->x(mCell[1]) - mFemObj->x(mCell[0]);
    Vector r2 = mFemObj->x(mCell[2]) - mFemObj->x(mCell[0]);
    Vector r3 = mFemObj->x(mCell[3]) - mFemObj->x(mCell[0]);

    Vector r4 = mFemObj->x(mCell[2]) - mFemObj->x(mCell[1]);
    Vector r5 = mFemObj->x(mCell[1]) - mFemObj->x(mCell[3]);

    mVolume = r1.cross(r2).dot(r3);

    mDnx[0] = r4.cross(r5) / mVolume;
    mDnx[1] = r2.cross(r3) / mVolume;
    mDnx[2] = r3.cross(r1) / mVolume;
    mDnx[3] = r1.cross(r2) / mVolume;

    mVolume /= 6.0;
}

void FiniteElement::updateDny()
{
    updateF(); // TODO: look if necessary
    Matrix3d F_inv = mF.inverse();
    for (size_t i = 0; i < 4; ++i) {
        mDny[i] = Vector(0,0,0);
        for (int a = 0; a < 3; ++a) {
            for (int k = 0; k < 3; ++k) {
                mDny[i](a) += mDnx[i](k) * F_inv(k,a);
            }
        }
    }
}
