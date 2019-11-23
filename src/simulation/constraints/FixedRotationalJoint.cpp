#include "FixedRotationalJoint.h"

#include <simulation/ImpulseConstraintSolver.h>
#include <simulation/SimulationObject.h>

#include <simulation/rigid/RigidBody.h>

#include <iostream>

FixedRotationalJoint::FixedRotationalJoint(
        const std::shared_ptr<RigidBody>& rbA,
        const std::shared_ptr<RigidBody>& rbB)
    : mRbA(rbA)
    , mRbB(rbB)
{
}

bool FixedRotationalJoint::references(SimulationObject* so)
{
    return so == mRbA.get() || so == mRbB.get();
}

void FixedRotationalJoint::initialize(double stepSize)
{
    mImpulseFactor = (mRbB->calculateL() + mRbA->calculateL()).inverse();

    // positoin error is calculated here
    Eigen::Quaterniond q1 = mRbA->getOrientation();
    Eigen::Quaterniond q2 = mRbB->getOrientation();

    Eigen::AngleAxisd deltaAngle(q1 * q2.inverse());

    mTargetOmegaRel = -(deltaAngle.angle() * deltaAngle.axis()) / stepSize;
}

bool FixedRotationalJoint::solve(double maxConstraintError)
{
    Eigen::Vector omegaRel =
            mRbA->getOrientationVelocity() -
            mRbB->getOrientationVelocity();

    Eigen::Vector deltaOmegaRel = mTargetOmegaRel - omegaRel;

    if (deltaOmegaRel.norm() < maxConstraintError)
    {
        return true;
    }

    Eigen::Vector impulse = mImpulseFactor * deltaOmegaRel;
    mRbA->applyOrientationImpulse(impulse);
    mRbB->applyOrientationImpulse(-impulse);

    return false;
}

void FixedRotationalJoint::accept(ConstraintVisitor& cv)
{
    cv.visit(this);
}
