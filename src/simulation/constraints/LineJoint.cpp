#include "LineJoint.h"

#include <simulation/ImpulseConstraintSolver.h>
#include <simulation/SimulationObject.h>

#include <simulation/rigid/RigidBody.h>

#include <iostream>


LineJoint::LineJoint(
        SimulationPointRef pointA,
        SimulationPointRef pointB,
        Eigen::Vector lineDirectionBS)
    : mPointA(pointA)
    , mPointB(pointB)
    , mLineDirectionBS(lineDirectionBS)
{

}

bool LineJoint::references(SimulationObject* so)
{
    return so == mPointA.getSimulationObject() ||
            so == mPointB.getSimulationObject();
}

void LineJoint::initialize(double stepSize)
{
    // calculate A
    Eigen::Vector A = mPointA.getPoint();
    mCurrentLineDir = calculateLineDirection();

    // Project point A so that it is the closest point on the line to B
    Eigen::Vector B = mPointB.getPoint();
    mCurrentAWS = A + (B - A).dot(mCurrentLineDir) * mCurrentLineDir;


    if (mPointA.getSimulationObject()->getType() == SimulationObject::Type::RIGID_BODY)
    {
        RigidBody* rb = static_cast<RigidBody*>(mPointA.getSimulationObject());
        mCurrentA = mCurrentAWS - rb->getCenterOfMass();
    }

    // now apply impulses just like in BallJoint
    mImpulseFactor = (ImpulseConstraintSolver::calculateK(mPointB) +
                      ImpulseConstraintSolver::calculateK(
                          mPointA.getSimulationObject(),
                          mCurrentA, mPointA.getIndex())).inverse();

    mTargetURel = -(mCurrentAWS - mPointB.getPoint()) / stepSize;

}

bool LineJoint::solve(double maxConstraintError)
{
    Eigen::Vector p1 = ImpulseConstraintSolver::calculateRelativePoint(
                mPointA.getSimulationObject(), mCurrentAWS);
    Eigen::Vector p2 = ImpulseConstraintSolver::calculateRelativePoint(
                mPointB.getSimulationObject(), mPointB.getPoint());

    Eigen::Vector uRel =
            ImpulseConstraintSolver::calculateSpeed(
                mPointA.getSimulationObject(), p1, mPointA.getIndex()) -
            ImpulseConstraintSolver::calculateSpeed(
                mPointB.getSimulationObject(), p2, mPointB.getIndex());

    // project uRel
    Eigen::Vector deltaURel = mTargetURel - uRel;

    Eigen::Vector deltaURelProj = deltaURel.dot(mCurrentLineDir) * mCurrentLineDir;
    deltaURel = deltaURel - deltaURelProj;

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

void LineJoint::accept(ConstraintVisitor& /*cv*/)
{

}

Eigen::Vector LineJoint::calculateLineDirection()
{
    // transform to world space if rigid
    Eigen::Vector lineDirection = mLineDirectionBS;
    if (mPointA.getSimulationObject()->getType() == SimulationObject::Type::RIGID_BODY)
    {
        RigidBody* rb = static_cast<RigidBody*>(mPointA.getSimulationObject());
        lineDirection = rb->getOrientation().toRotationMatrix() * lineDirection;
    }
    return lineDirection;
}
