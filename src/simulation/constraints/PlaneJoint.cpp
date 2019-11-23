#include "PlaneJoint.h"

#include <simulation/ImpulseConstraintSolver.h>
#include <simulation/SimulationObject.h>

#include <simulation/rigid/RigidBody.h>

#include <iostream>


PlaneJoint::PlaneJoint(
        SimulationPointRef pointA,
        SimulationPointRef pointB,
        Eigen::Vector lineDir1BS,
        Eigen::Vector lineDir2BS)
    : mPointA(pointA)
    , mPointB(pointB)
    , mLineDir1BS(lineDir1BS)
    , mLineDir2BS(lineDir2BS)
{

}

bool PlaneJoint::references(SimulationObject* so)
{
    return so == mPointA.getSimulationObject().get() ||
            so == mPointB.getSimulationObject().get();
}

void PlaneJoint::initialize(double stepSize)
{
    // calculate A
    Eigen::Vector A = mPointA.getPoint();
    mCurrentLineDir1 = calculateLineDirection(mLineDir1BS);
    mCurrentLineDir2 = calculateLineDirection(mLineDir2BS);

    // Project point A so that it is the closest point on the line to B
    Eigen::Vector B = mPointB.getPoint();
    Eigen::Vector rABS1 = (B - A).dot(mCurrentLineDir1) * mCurrentLineDir1;
    Eigen::Vector rABS2 = (B - A).dot(mCurrentLineDir2) * mCurrentLineDir2;
    mCurrentAWS = A + rABS1 + rABS2;

    if (mPointA.getSimulationObject()->getType() == SimulationObject::Type::RIGID_BODY)
    {
        RigidBody* rb = static_cast<RigidBody*>(mPointA.getSimulationObject().get());
        mCurrentA = mCurrentAWS - rb->getCenterOfMass();
    }

    // now apply impulses just like in BallJoint
    mImpulseFactor = (ImpulseConstraintSolver::calculateK(mPointB) +
                      ImpulseConstraintSolver::calculateK(
                          mPointA.getSimulationObject().get(),
                          mCurrentA, mPointA.getIndex())).inverse();

    mTargetURel = -(mCurrentAWS - B) / stepSize;

    mPoint1 = ImpulseConstraintSolver::calculateRelativePoint(
                mPointA.getSimulationObject().get(), mCurrentAWS);
    mPoint2 = ImpulseConstraintSolver::calculateRelativePoint(
                mPointB.getSimulationObject().get(), B);
}

bool PlaneJoint::solve(double maxConstraintError)
{
//    Eigen::Vector p1 = ImpulseConstraintSolver::calculateRelativePoint(
//                mPointA.getSimulationObject().get(), mCurrentAWS);
//    Eigen::Vector p2 = ImpulseConstraintSolver::calculateRelativePoint(
//                mPointB.getSimulationObject().get(), mPointB.getPoint());

    Eigen::Vector uRel =
            ImpulseConstraintSolver::calculateSpeed(
                mPointA.getSimulationObject().get(), mPoint1, mPointA.getIndex()) -
            ImpulseConstraintSolver::calculateSpeed(
                mPointB.getSimulationObject().get(), mPoint2, mPointB.getIndex());

    // project uRel
    Eigen::Vector deltaURel = mTargetURel - uRel;

    Eigen::Vector deltaURel1BS = deltaURel.dot(mCurrentLineDir1) * mCurrentLineDir1;
    Eigen::Vector deltaURel2BS = deltaURel.dot(mCurrentLineDir2) * mCurrentLineDir2;
    Eigen::Vector deltaURelProj = deltaURel1BS + deltaURel2BS;

    deltaURel = deltaURel - deltaURelProj;

    if (deltaURel.norm() < maxConstraintError)
    {
        return true;
    }

    Eigen::Vector impulse = mImpulseFactor * deltaURel;

    ImpulseConstraintSolver::applyImpulse(
                mPointA.getSimulationObject().get(), impulse,
                mPoint1, mPointA.getIndex());
    ImpulseConstraintSolver::applyImpulse(
                mPointB.getSimulationObject().get(), -impulse,
                mPoint2, mPointB.getIndex());

    return false;
}

void PlaneJoint::accept(ConstraintVisitor& cv)
{
    cv.visit(this);
}

Eigen::Vector PlaneJoint::calculateLineDirection(const Eigen::Vector& lineBS)
{
    // transform to world space if rigid
    Eigen::Vector lineDirection = lineBS;
    if (mPointA.getSimulationObject()->getType() == SimulationObject::Type::RIGID_BODY)
    {
        RigidBody* rb = static_cast<RigidBody*>(mPointA.getSimulationObject().get());
        lineDirection = rb->getOrientation().toRotationMatrix() * lineDirection;
    }
    return lineDirection;
}
