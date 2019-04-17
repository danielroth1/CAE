#include "AxisRotationalJoint.h"

#include <simulation/ImpulseConstraintSolver.h>
#include <simulation/SimulationObject.h>

#include <simulation/rigid/RigidBody.h>


AxisRotationalJoint::AxisRotationalJoint(
        RigidBody* rbA,
        RigidBody* rbB,
        Eigen::Vector axisBS)
    : mRbA(rbA)
    , mRbB(rbB)
    , mAxisBS(axisBS)
{
    mAxisBS.normalize();
}

bool AxisRotationalJoint::references(SimulationObject* so)
{
    return so == mRbA || so == mRbB;
}

void AxisRotationalJoint::initialize(double stepSize)
{
    // position error
    Eigen::Quaterniond q1 = mRbA->getOrientation();
    Eigen::Quaterniond q2 = mRbB->getOrientation();

    // calculate delta
    Eigen::Vector a1 = mRbA->getOrientation().toRotationMatrix() * mAxisBS;
    Eigen::Vector a2 = mRbB->getOrientation().toRotationMatrix() * mAxisBS;
    Eigen::Vector n = a1.cross(a2);
    double l = n.norm();
    if (l > 1e-8)
        n /= l;

    double angle = std::atan2(l, a1.dot(a2));
    mProjMatrix = ImpulseConstraintSolver::calculateProjectionMatrix(a1);

    // calculate delta
    Eigen::Vector2d delta = mProjMatrix * (angle * n);

    mImpulseFactor =
            (mProjMatrix *
            (mRbA->calculateL() + mRbB->calculateL()) *
            mProjMatrix.transpose()).inverse();

    mTargetOmegaRel = delta / stepSize;
}

bool AxisRotationalJoint::solve(double maxConstraintError)
{
    Eigen::Vector omegaRel = mRbA->getOrientationVelocity() -
            mRbB->getOrientationVelocity();

    Eigen::Vector2d deltaOmegaRel = mTargetOmegaRel - mProjMatrix * omegaRel;

    if (deltaOmegaRel.norm() < maxConstraintError)
    {
        return true;
    }

    Eigen::Vector impulse = mProjMatrix.transpose() * (mImpulseFactor * deltaOmegaRel);
    mRbA->applyOrientationImpulse(impulse);
    mRbB->applyOrientationImpulse(-impulse);

    return false;
}

void AxisRotationalJoint::accept(ConstraintVisitor& cv)
{

}
