#include "BallJoint.h"
#include "ConstraintVisitor.h"

#include <simulation/ImpulseConstraintSolver.h>

#include <iostream>

BallJoint::BallJoint(SimulationPointRef pointA, SimulationPointRef pointB)
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

    // comment in to print the norm
//    std::cout << "position error = " << mTargetURel.norm() <<
//                 ", p1 = " << mPointA.getPoint().transpose() <<
//                 ", p2 = " << mPointB.getPoint().transpose() << "\n";
}

bool BallJoint::solve(double maxConstraintError)
{
    Eigen::Vector p1 = ImpulseConstraintSolver::calculateRelativePoint(
                mPointA.getSimulationObject(), mPointA.getPoint());
    Eigen::Vector p2 = ImpulseConstraintSolver::calculateRelativePoint(
                mPointB.getSimulationObject(), mPointB.getPoint());

    Eigen::Vector uRel =
            ImpulseConstraintSolver::calculateSpeed(
                mPointA.getSimulationObject(), p1, mPointA.getIndex()) -
            ImpulseConstraintSolver::calculateSpeed(
                mPointB.getSimulationObject(), p2, mPointB.getIndex());

    Eigen::Vector deltaURel = mTargetURel - uRel;

    std::cout << "norm = " << deltaURel.norm() << "\n";
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

void BallJoint::accept(ConstraintVisitor& cv)
{
    cv.visit(this);
}

bool BallJoint::references(SimulationObject* so)
{
    return so == mPointA.getSimulationObject() ||
            so == mPointB.getSimulationObject();
}
