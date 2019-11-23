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

    mPoint1 = ImpulseConstraintSolver::calculateRelativePoint(
                mPointA.getSimulationObject().get(), mPointA.getPoint());
    mPoint2 = ImpulseConstraintSolver::calculateRelativePoint(
                mPointB.getSimulationObject().get(), mPointB.getPoint());

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
    Eigen::Vector v1 = ImpulseConstraintSolver::calculateSpeed(
                mPointA.getSimulationObject().get(), mPoint1, mPointA.getIndex());
    Eigen::Vector v2 = ImpulseConstraintSolver::calculateSpeed(
                mPointB.getSimulationObject().get(), mPoint2, mPointB.getIndex());

    Eigen::Vector uRel = v1 - v2;

    // project uRel on sphere
    // For some reason advancing the step isn't necesssary here.
    Eigen::Vector p1Next = mPoint1; // + mStepSize * v1;
    Eigen::Vector p2Next = mPoint2; // + mStepSize * v2;

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
                mPointA.getSimulationObject().get(), impulse, mPoint1, mPointA.getIndex());
    ImpulseConstraintSolver::applyImpulse(
                mPointB.getSimulationObject().get(), -impulse, mPoint2, mPointB.getIndex());

    return false;
}

void DistanceJoint::accept(ConstraintVisitor& cv)
{
    cv.visit(this);
}

bool DistanceJoint::references(SimulationObject* so)
{
    return so == mPointA.getSimulationObject().get() ||
            so == mPointB.getSimulationObject().get();
}
