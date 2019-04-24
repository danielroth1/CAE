#include "DistanceJoint.h"
#include "ConstraintVisitor.h"

#include <simulation/ImpulseConstraintSolver.h>

#include <iostream>

DistanceJoint::DistanceJoint(
        SimulationPointRef pointA,
        SimulationPointRef pointB,
        double distance)
    : mPointA(pointA)
    , mPointB(pointB)
    , mDistance(distance)
{
    mTargetURel = Eigen::Vector::Zero();
}

const Eigen::Vector& DistanceJoint::getTargetURel() const
{
    return mTargetURel;
}

const Eigen::Vector& DistanceJoint::getSumOfAllAppliedImpulses() const
{
    return mSumOfAllAppliedImpulses;
}

void DistanceJoint::setSumOfAllAppliedImpulses(const Eigen::Vector& impulses)
{
    mSumOfAllAppliedImpulses = impulses;
}

void DistanceJoint::initialize(double stepSize)
{
    mStepSize = stepSize;
    mImpulseFactor = (ImpulseConstraintSolver::calculateK(mPointB) +
                      ImpulseConstraintSolver::calculateK(mPointA)).inverse();

    mTargetURel = Eigen::Vector::Zero();

    if (mDistance > 1e-10)
    {
        Eigen::Vector direction = mPointA.getPoint() - mPointB.getPoint();
        if (direction.norm() < 1e-10)
            direction = Eigen::Vector(1.0, 0.0, 0.0);
        else
            direction.normalize();

        mTargetURel = mDistance * direction / stepSize;
    }

    mTargetURel -= (mPointA.getPoint() - mPointB.getPoint() ) / stepSize;
}

bool DistanceJoint::solve(double maxConstraintError)
{
    Eigen::Vector p1 = ImpulseConstraintSolver::calculateRelativePoint(
                mPointA.getSimulationObject(), mPointA.getPoint());
    Eigen::Vector p2 = ImpulseConstraintSolver::calculateRelativePoint(
                mPointB.getSimulationObject(), mPointB.getPoint());

    Eigen::Vector v1 = ImpulseConstraintSolver::calculateSpeed(
                mPointA.getSimulationObject(), p1, mPointA.getIndex());
    Eigen::Vector v2 = ImpulseConstraintSolver::calculateSpeed(
                mPointB.getSimulationObject(), p2, mPointB.getIndex());

    Eigen::Vector uRel = v1 - v2;

    // project uRel on sphere
    Eigen::Vector p1Next = mPointA.getPoint();// + mStepSize * v1;
    Eigen::Vector p2Next = mPointB.getPoint();// + mStepSize * v2;

    Eigen::Vector direction = p2Next - p1Next;
    if (direction.norm() < 1e-10)
        direction = Eigen::Vector(1.0, 0.0, 0.0);
    else
        direction.normalize();

    Eigen::Vector deltaURel = mTargetURel - uRel;
    deltaURel = deltaURel.dot(direction) * direction;

    if (deltaURel.norm() < maxConstraintError)
    {
        return true;
    }

    Eigen::Vector impulse = mImpulseFactor * deltaURel;

    ImpulseConstraintSolver::applyImpulse(
                mPointA.getSimulationObject(), impulse, p1, mPointA.getIndex());
    ImpulseConstraintSolver::applyImpulse(
                mPointB.getSimulationObject(), -impulse, p2, mPointB.getIndex());

    return false;
}

void DistanceJoint::accept(ConstraintVisitor& /*cv*/)
{

}

bool DistanceJoint::references(const std::shared_ptr<SimulationObject>& so)
{
    return so == mPointA.getSimulationObject() ||
            so == mPointB.getSimulationObject();
}
