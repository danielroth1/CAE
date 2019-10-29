#include "FiniteElement.h"

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
    mQ.setIdentity();
}

void FiniteElement::setMaterial(ElasticMaterial material)
{
    mMaterial = material;
    mStiffnessMatrixDirty = true;
}

void FiniteElement::initialize()
{
    updateDnx();

    // Initialize mQ with the correct rotation calculated with the slow singular
    // value decomposition. Use that result with the iterative approach later.
    updateRotation();
    mQ = Eigen::Quaterniond(mR);

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

void FiniteElement::updateRotationFast(size_t maxIterations, double tolerance)
{
    updateF();

    for (size_t i = 0; i < maxIterations; ++i)
    {
        Eigen::Matrix3d R = mQ.toRotationMatrix();
        double s = std::abs(R.col(0).dot(mF.col(0)) +
                            R.col(1).dot(mF.col(1)) +
                            R.col(2).dot(mF.col(2)));

        if (s < 1e-12)
            break;

        double sInv = 1.0 / s + tolerance;
        Eigen::Vector3d v = R.col(0).cross(mF.col(0)) +
                            R.col(1).cross(mF.col(1)) +
                            R.col(2).cross(mF.col(2));

        Eigen::Vector3d omega = sInv * v;
        double w = omega.norm();

        if (w < tolerance)
            break;

        Eigen::Quaterniond omegaQ =
                Eigen::Quaterniond(Eigen::AngleAxisd(w, 1.0 / w * omega));

        mQ = omegaQ * mQ;
    }

    mR = mQ.toRotationMatrix();
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
    for (size_t a = 0; a < 4; ++a)
    {
        mForces[a].setZero();
        for (size_t b = 0; b < 4; ++b)
        {
            mForces[a] += mR * mK[a][b] *
                    (mR.transpose() * y(b) - x(b));
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

    // Sometimes tetrahedrons can be so degenerated that their volume is zero.
    if (mVolume < 1e-10)
        mVolume = 1e-5;

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
