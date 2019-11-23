#include "CollisionConstraint.h"
#include "ConstraintVisitor.h"

#include <simulation/collision_detection/narrow/Collision.h>

#include <simulation/ImpulseConstraintSolver.h>

#include <iostream>

CollisionConstraint::CollisionConstraint(
        Collision& collision,
        double restitution,
        double cFrictionDynamic,
        double cFrictionStatic)
    : mCollision(collision)
    , mRestitution(restitution)
    , mCFrictionDynamic(cFrictionDynamic)
    , mCFrictionStatic(cFrictionStatic)
{

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
    mSumOfAllAppliedImpulses = Eigen::Vector::Zero();
    mSumFrictionImpulses = Eigen::Vector::Zero();

    mPoint1 = ImpulseConstraintSolver::calculateRelativePoint(
                mCollision.getSimulationObjectA(), mCollision.getPointA());
    mPoint2 = ImpulseConstraintSolver::calculateRelativePoint(
                mCollision.getSimulationObjectB(), mCollision.getPointB());

    const Eigen::Vector& n = mCollision.getNormal();

    Eigen::Vector u1 = ImpulseConstraintSolver::calculateSpeed(
                mCollision.getSimulationObjectA(), mPoint1, mCollision.getVertexIndexA());
    Eigen::Vector u2 = ImpulseConstraintSolver::calculateSpeed(
                mCollision.getSimulationObjectB(), mPoint2, mCollision.getVertexIndexB());

    // TODO:
    // (mCollision.getPointA() - mCollision.getPointB()).dot(n) is the
    // penetration depth of the collision in the predictor step.
    // This is not the distance from the contact points in the current step!
    double positionCorrection = 0.0;//0.2 * std::max(0.0, (mCollision.getPointA() - mCollision.getPointB()).dot(n) / stepSize);
//    double positionCorrection = -std::max(0.0, (mCollision.getPointB() - mCollision.getPointA()).dot(n) / stepSize);
    if (positionCorrection > 1e-10)
        std::cout << "position correction = " << positionCorrection << "\n";
    Eigen::Vector uRel = u1 - u2;

    mTargetUNormalRel = (-mRestitution * uRel.dot(n) + positionCorrection) * n;

    if (/*mCollision.isInside() && */mTargetUNormalRel.dot(n) < 0)
    {
        mTargetUNormalRel *= -1.0 / mRestitution;
    }

    mK = ImpulseConstraintSolver::calculateK(
                mCollision.getSimulationObjectA(), mPoint1, mCollision.getVertexIndexA()) +
            ImpulseConstraintSolver::calculateK(
                mCollision.getSimulationObjectB(), mPoint2, mCollision.getVertexIndexB());

    mImpulseFactor = 1 / (n.transpose() * mK * n);
}

bool CollisionConstraint::solve(double maxConstraintError)
{
    const Eigen::Vector& n = mCollision.getNormal();

    u1 = ImpulseConstraintSolver::calculateSpeed(
                mCollision.getSimulationObjectA(),
                mPoint1,
                mCollision.getVertexIndexA());

    u2 = ImpulseConstraintSolver::calculateSpeed(
                mCollision.getSimulationObjectB(),
                mPoint2,
                mCollision.getVertexIndexB());

    uRel = u1 - u2;

    uRelN = uRel.dot(n) * n;

    deltaUNormalRel = mTargetUNormalRel - uRelN;
    if (deltaUNormalRel.squaredNorm() < maxConstraintError * maxConstraintError)
    {
        return true;
    }
    impulse = mImpulseFactor * deltaUNormalRel;
    if (mCollision.getNormal().dot(mSumOfAllAppliedImpulses + impulse) < 0)
    {
        impulse = -mSumOfAllAppliedImpulses;
    }
    mSumOfAllAppliedImpulses += impulse;

    ImpulseConstraintSolver::applyImpulse(
                mCollision.getSimulationObjectA(), impulse,
                mPoint1, mCollision.getVertexIndexA());

    ImpulseConstraintSolver::applyImpulse(
                mCollision.getSimulationObjectB(), -impulse,
                mPoint2, mCollision.getVertexIndexB());

    // Friction
    // Don't apply friction if two objects are currently penetrating so that
    // the process of resolving the interpenetration isn't slowed down.
    // do this in collider?

    // this is not the best way of doing it. instead disable friction between
    // all contacts if there is at least one "isInside" contact
    if (!mCollision.isInside() &&
        (mCFrictionStatic > 1e-10 ||
        mCFrictionDynamic > 1e-10))
    {
        uRelT = uRel - uRelN;
        if (uRelT.norm() > 1e-8)
        {
            t = uRelT.normalized();
            frictionImpulseMax = - 1 / (t.transpose() * mK * t) * uRelT;

            if (mSumFrictionImpulses.norm() <= mCFrictionStatic * mSumOfAllAppliedImpulses.norm())
            {
                // static friction
                frictionImpulse = frictionImpulseMax;
            }
            else
            {
                // dynamic friction
                // This is the equation in the paper but the other one is used
                // in IBDS.
//                frictionImpulse = -mCFrictionDynamic * impulse.norm() * t;
                frictionImpulse = -mCFrictionDynamic * impulse.dot(n) * t;

                if (frictionImpulse.dot(t) < frictionImpulseMax.dot(t))
                {
                    frictionImpulse = frictionImpulseMax;
                }

            }

            mSumFrictionImpulses += frictionImpulse;

            ImpulseConstraintSolver::applyImpulse(
                        mCollision.getSimulationObjectA(), frictionImpulse,
                        mPoint1, mCollision.getVertexIndexA());

            ImpulseConstraintSolver::applyImpulse(
                        mCollision.getSimulationObjectB(), -frictionImpulse,
                        mPoint2, mCollision.getVertexIndexB());
        }
    }

    return false;
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
