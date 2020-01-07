#include "LineJoint.h"

#include <simulation/ImpulseConstraintSolver.h>
#include <simulation/SimulationObject.h>

#include <simulation/rigid/RigidBody.h>

#include <iostream>


LineJoint::LineJoint(
        SimulationPointRef pointA,
        SimulationPointRef pointB,
        Eigen::Vector lineDirectionBS)
    : mPointARef(pointA)
    , mPointBRef(pointB)
    , mLineDirectionBS(lineDirectionBS)
{
    mPointARef.setUpdatePolicy(SimulationPointRef::UpdatePolicy::ON_UPDATE_CALL);
    mPointBRef.setUpdatePolicy(SimulationPointRef::UpdatePolicy::ON_UPDATE_CALL);
}

bool LineJoint::references(SimulationObject* so)
{
    return so == mPointARef.getSimulationObject().get() ||
            so == mPointBRef.getSimulationObject().get();
}

void LineJoint::initialize(double stepSize)
{
    mPointARef.update();
    mPointBRef.update();

    // calculate A
    Eigen::Vector A = mPointARef.getPoint();
    mCurrentLineDir = calculateLineDirection();

    // Project point A so that it is the closest point on the line to B
    mPointBWS = mPointBRef.getPoint();
    mPointAWS = A + (mPointBWS - A).dot(mCurrentLineDir) * mCurrentLineDir;

    if (mPointARef.getSimulationObject()->getType() == SimulationObject::Type::RIGID_BODY)
    {
        RigidBody* rb = static_cast<RigidBody*>(mPointARef.getSimulationObject().get());
        mPointABS = mPointAWS - rb->getCenterOfMass();
    }

    // now apply impulses just like in BallJoint
    mImpulseFactor = (ImpulseConstraintSolver::calculateK(mPointBRef) +
                      ImpulseConstraintSolver::calculateK(
                          mPointARef.getSimulationObject().get(),
                          mPointABS, mPointARef.getIndex())).inverse();

    mTargetURel = -(mPointAWS - mPointBWS) / stepSize;

    mPointBBS = ImpulseConstraintSolver::calculateRelativePoint(
                mPointBRef.getSimulationObject().get(), mPointBWS);
}

bool LineJoint::solve(double maxConstraintError)
{
    Eigen::Vector uRel =
            ImpulseConstraintSolver::calculateSpeed(
                mPointARef.getSimulationObject().get(), mPointABS, mPointARef.getIndex()) -
            ImpulseConstraintSolver::calculateSpeed(
                mPointBRef.getSimulationObject().get(), mPointBBS, mPointBRef.getIndex());

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
                mPointARef.getSimulationObject().get(), impulse,
                mPointABS, mPointARef.getIndex());
    ImpulseConstraintSolver::applyImpulse(
                mPointBRef.getSimulationObject().get(), -impulse,
                mPointBBS, mPointBRef.getIndex());

    return false;
}

void LineJoint::accept(ConstraintVisitor& cv)
{
    cv.visit(this);
}

Eigen::Vector LineJoint::calculateLineDirection()
{
    // transform to world space if rigid
    Eigen::Vector lineDirection = mLineDirectionBS;
    if (mPointARef.getSimulationObject()->getType() == SimulationObject::Type::RIGID_BODY)
    {
        RigidBody* rb = static_cast<RigidBody*>(mPointARef.getSimulationObject().get());
        lineDirection = rb->getOrientation() * lineDirection;
    }
    return lineDirection;
}
