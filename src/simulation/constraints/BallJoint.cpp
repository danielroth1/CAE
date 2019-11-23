#include "BallJoint.h"
#include "ConstraintVisitor.h"

#include <simulation/ImpulseConstraintSolver.h>

#include <iostream>

BallJoint::BallJoint(
        SimulationPointRef pointA,
        SimulationPointRef pointB)
    : mPointA(pointA)
    , mPointB(pointB)
{
    mTargetURel = Eigen::Vector::Zero();
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

void BallJoint::initialize(double stepSize)
{
    mImpulseFactor = (ImpulseConstraintSolver::calculateK(mPointB) +
                      ImpulseConstraintSolver::calculateK(mPointA)).inverse();

    mTargetURel = -(mPointA.getPoint() - mPointB.getPoint()) / stepSize;

    mPoint1 = ImpulseConstraintSolver::calculateRelativePoint(
                mPointA.getSimulationObject().get(), mPointA.getPoint());
    mPoint2 = ImpulseConstraintSolver::calculateRelativePoint(
                mPointB.getSimulationObject().get(), mPointB.getPoint());

    // comment in to print the norm
//    std::cout << "position error = " << mTargetURel.norm() <<
//                 ", p1 = " << mPointA.getPoint().transpose() <<
//                 ", p2 = " << mPointB.getPoint().transpose() << "\n";
}

bool BallJoint::solve(double maxConstraintError)
{
    Eigen::Vector uRel =
            ImpulseConstraintSolver::calculateSpeed(
                mPointA.getSimulationObject().get(), mPoint1, mPointA.getIndex()) -
            ImpulseConstraintSolver::calculateSpeed(
                mPointB.getSimulationObject().get(), mPoint2, mPointB.getIndex());

    Eigen::Vector deltaURel = mTargetURel - uRel;

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

void BallJoint::accept(ConstraintVisitor& cv)
{
    cv.visit(this);
}

bool BallJoint::references(SimulationObject* so)
{
    return so == mPointA.getSimulationObject().get() ||
            so == mPointB.getSimulationObject().get();
}
