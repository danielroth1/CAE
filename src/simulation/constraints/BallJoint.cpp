#include "BallJoint.h"

BallJoint::BallJoint(SimulationPointRef pointA, SimulationPointRef pointB)
    : mPointA(pointA)
    , mPointB(pointB)
{

}

const Eigen::Vector& BallJoint::getTargetURel() const
{
    return mTargetURel;
}

const Eigen::Vector& BallJoint::getSumOfAllAppliedImpulses() const
{
    return mSumOfAllAppliedImpulses;
}

void BallJoint::setSumOfAllAppliedImpulses(const Eigen::Vector& impulses)
{
    mSumOfAllAppliedImpulses = impulses;
}

double BallJoint::getImpulseFactor()
{
    return mImpulseFactor;
}

void BallJoint::setImpulseFactor(double impulseFactor)
{
    mImpulseFactor = impulseFactor;
}

void BallJoint::accept(ConstraintVisitor& cv)
{

}
