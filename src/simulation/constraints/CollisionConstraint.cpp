#include "CollisionConstraint.h"
#include "ConstraintVisitor.h"

#include <simulation/collision_detection/narrow/Collision.h>

#include <simulation/ImpulseConstraintSolver.h>

#include <iostream>

#include <simulation/fem/FEMObject.h>

CollisionConstraint::CollisionConstraint(
        Collision& collision,
        double restitution,
        double positionCorrectionFactor,
        double collisionMargin)
    : mCollision(collision)
    , mRestitution(restitution)
    , mPositionCorrectionFactor(positionCorrectionFactor)
    , mCollisionMargin(collisionMargin)
{
    mCFrictionStatic = std::sqrt(
                (collision.getSimulationObjectA()->getFrictionStatic() *
                 collision.getSimulationObjectB()->getFrictionStatic()));

    mCFrictionDynamic = std::sqrt(
                (collision.getSimulationObjectA()->getFrictionDynamic() *
                 collision.getSimulationObjectB()->getFrictionDynamic()));
}

CollisionConstraint::~CollisionConstraint()
{

}

Collision& CollisionConstraint::getCollision()
{
    return mCollision;
}

const Eigen::Vector& CollisionConstraint::getTargetUNormalRel() const
{
    return mTargetUNormalRel;
}

const Eigen::Vector& CollisionConstraint::getSumOfAllAppliedImpulses() const
{
    return mSumOfAllAppliedImpulses;
}

void CollisionConstraint::initialize(double stepSize)
{
    mSticking = false;
    mSumOfAllAppliedImpulses = Eigen::Vector::Zero();
    mSumFrictionImpulses = Eigen::Vector::Zero();

    mPoint1 = ImpulseConstraintSolver::calculateRelativePoint(
                mCollision.getSimulationObjectA(), mCollision.getPointA());
    mPoint2 = ImpulseConstraintSolver::calculateRelativePoint(
                mCollision.getSimulationObjectB(), mCollision.getPointB());

    const Eigen::Vector& n = mCollision.getNormal();

    u1 = ImpulseConstraintSolver::calculateSpeed(
                mCollision.getSimulationObjectA(), mPoint1,
                mCollision.getBarycentricCoordiantesA(), mCollision.getElementIdA());
    u2 = ImpulseConstraintSolver::calculateSpeed(
                mCollision.getSimulationObjectB(), mPoint2,
                mCollision.getBarycentricCoordiantesB(), mCollision.getElementIdB());

    // Correction of the position error of the previous time step.
    Eigen::Vector posPrevA = mCollision.calculatePositionPreviousA();
    Eigen::Vector posPrevB = mCollision.calculatePositionPreviousB();

    double posError = (posPrevA - posPrevB).dot(n);
    posError = std::min(posError - mCollisionMargin, 0.0);
    double positionCorrection = -mPositionCorrectionFactor * posError / stepSize;

    uRel = u1 - u2;

    mTargetUNormalRel = (-mRestitution * uRel.dot(n) + positionCorrection) * n;

    if (/*mCollision.isInside() && */mTargetUNormalRel.dot(n) < 0)
    {
        mTargetUNormalRel *= -1.0 / mRestitution;
    }

    mK = ImpulseConstraintSolver::calculateK(
                mCollision.getSimulationObjectA(), mPoint1,
                mCollision.getBarycentricCoordiantesA(), mCollision.getElementIdA()) +
            ImpulseConstraintSolver::calculateK(
                mCollision.getSimulationObjectB(), mPoint2,
                mCollision.getBarycentricCoordiantesB(), mCollision.getElementIdB());

    mImpulseFactor = 1 / (n.transpose() * mK * n);

}

bool CollisionConstraint::solve(double maxConstraintError)
{
    const Eigen::Vector& n = mCollision.getNormal();

    u1 = ImpulseConstraintSolver::calculateSpeed(
                mCollision.getSimulationObjectA(), mPoint1,
                mCollision.getBarycentricCoordiantesA(), mCollision.getElementIdA());
    u2 = ImpulseConstraintSolver::calculateSpeed(
                mCollision.getSimulationObjectB(), mPoint2,
                mCollision.getBarycentricCoordiantesB(), mCollision.getElementIdB());

    uRel = u1 - u2;

    uRelN = uRel.dot(n) * n;

    deltaUNormalRel = mTargetUNormalRel - uRelN;

    bool appliedCollisionImpulse;
    if (deltaUNormalRel.squaredNorm() < maxConstraintError * maxConstraintError)
    {
        appliedCollisionImpulse = false;
    }
    else
    {
        appliedCollisionImpulse = true;
        impulse = mImpulseFactor * deltaUNormalRel;
        if (mCollision.getNormal().dot(mSumOfAllAppliedImpulses + impulse) < 0)
        {
            impulse = -mSumOfAllAppliedImpulses;
        }
        mSumOfAllAppliedImpulses += impulse;

        ImpulseConstraintSolver::applyImpulse(
                    mCollision.getSimulationObjectA(), impulse,
                    mPoint1, mCollision.getBarycentricCoordiantesA(),
                    mCollision.getElementIdA());

        ImpulseConstraintSolver::applyImpulse(
                    mCollision.getSimulationObjectB(), -impulse,
                    mPoint2, mCollision.getBarycentricCoordiantesB(),
                    mCollision.getElementIdB());

    }

    // Friction
    // Don't apply friction if two objects are currently penetrating so that
    // the process of resolving the interpenetration isn't slowed down.

    // Note: This is not the best way of doing it. instead disable friction
    // between all contacts if there is at least one "isInside" contact.

    bool appliedFriction = false;

    if (!mCollision.isInside() &&
        (mCFrictionStatic > 1e-15 ||
        mCFrictionDynamic > 1e-15))
    {
        // recalculate uRel after the collision impulse was applied
        u1 = ImpulseConstraintSolver::calculateSpeed(
                    mCollision.getSimulationObjectA(), mPoint1,
                    mCollision.getBarycentricCoordiantesA(), mCollision.getElementIdA());
        u2 = ImpulseConstraintSolver::calculateSpeed(
                    mCollision.getSimulationObjectB(), mPoint2,
                    mCollision.getBarycentricCoordiantesB(), mCollision.getElementIdB());

        uRel = u1 - u2;
        uRelN = uRel.dot(n) * n;
        uRelT = uRel - uRelN;

        if (uRelT.norm() > 1e-8)
        {
            Eigen::Vector tangent = uRelT.normalized();

            frictionImpulseMax = - 1 / (tangent.transpose() * mK * tangent) * uRelT;

            // This is the equation from the paper.
            frictionImpulse = -mCFrictionDynamic * impulse.dot(n) * tangent;

            if (mSticking || (mSumOfAllAppliedImpulses.norm() > 1e-8 &&
                              (mSumFrictionImpulses + frictionImpulse).norm() <= mCFrictionStatic * mSumOfAllAppliedImpulses.norm()))
            {
                // Static friction
                mSticking = true;
                frictionImpulse = frictionImpulseMax;
            }
            else if (appliedCollisionImpulse)
            {
                // dynamic friction
                if (frictionImpulse.dot(tangent) < frictionImpulseMax.dot(tangent))
                {
                    frictionImpulse = frictionImpulseMax;
                    mSticking = true;
                }
            }

            mSumFrictionImpulses += frictionImpulse;

            if (frictionImpulse.squaredNorm() > maxConstraintError * maxConstraintError)
            {
                appliedFriction = true;

                ImpulseConstraintSolver::applyImpulse(
                            mCollision.getSimulationObjectA(), frictionImpulse,
                            mPoint1, mCollision.getBarycentricCoordiantesA(),
                            mCollision.getElementIdA());

                ImpulseConstraintSolver::applyImpulse(
                            mCollision.getSimulationObjectB(), -frictionImpulse,
                            mPoint2, mCollision.getBarycentricCoordiantesB(),
                            mCollision.getElementIdB());
            }
        }
    }

    return !appliedCollisionImpulse && !appliedFriction;
}

void CollisionConstraint::accept(ConstraintVisitor& cv)
{
    cv.visit(this);
}

bool CollisionConstraint::references(SimulationObject* so)
{
    return so == mCollision.getSimulationObjectA() ||
            so == mCollision.getSimulationObjectB();
}
