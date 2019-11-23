#include "DoubleAxisRotationalJoint.h"

#include <simulation/ImpulseConstraintSolver.h>
#include <simulation/SimulationObject.h>

#include <simulation/rigid/RigidBody.h>

#include <iostream>

DoubleAxisRotationalJoint::DoubleAxisRotationalJoint(
        const std::shared_ptr<RigidBody>& rbA,
        const std::shared_ptr<RigidBody>& rbB,
        Eigen::Vector axis1BS,
        Eigen::Vector axis2BS)
    : mRbA(rbA)
    , mRbB(rbB)
    , mAxis1BS(axis1BS)
    , mAxis2BS(axis2BS)
{
    mAxis1BS.normalize();
    mAxis2BS.normalize();
}

bool DoubleAxisRotationalJoint::references(SimulationObject* so)
{
    return so == mRbA.get() || so == mRbB.get();
}

void DoubleAxisRotationalJoint::initialize(double stepSize)
{
    // position error
    Eigen::Quaterniond q1 = mRbA->getOrientation();
    Eigen::Quaterniond q2 = mRbB->getOrientation();

    // calculate delta
    Eigen::Vector a1 = mRbA->getOrientation().toRotationMatrix() * mAxis1BS;
    Eigen::Vector a2 = mRbB->getOrientation().toRotationMatrix() * mAxis2BS;
    Eigen::Vector n = a1.cross(a2);
    double l = n.norm();
    if (l > 1e-8)
        n /= l;

    double angle = std::atan2(l, a1.dot(a2));
    mProjMatrixT = ImpulseConstraintSolver::calculateProjectionMatrix(a1, a2);
    mProjMatrix = mProjMatrixT.transpose();

    // calculate delta
    double delta = mProjMatrix * (angle * n);

    // 1x3 * 3x3 * 3x1
    mImpulseFactor =
            1 / (mProjMatrix * (mRbA->calculateL() + mRbB->calculateL()) *
                 mProjMatrixT);

    mTargetOmegaRel = delta / stepSize;
}

bool DoubleAxisRotationalJoint::solve(double maxConstraintError)
{
    Eigen::Vector omegaRel = mRbA->getOrientationVelocity() -
            mRbB->getOrientationVelocity();

    double deltaOmegaRel = mTargetOmegaRel - mProjMatrix * omegaRel;

    if (deltaOmegaRel < maxConstraintError)
    {
        return true;
    }

    Eigen::Vector impulse = mProjMatrix * (mImpulseFactor * deltaOmegaRel);
    mRbA->applyOrientationImpulse(impulse);
    mRbB->applyOrientationImpulse(-impulse);

    return false;
}

void DoubleAxisRotationalJoint::accept(ConstraintVisitor& cv)
{
    cv.visit(this);
}
