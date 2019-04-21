#include "CollisionConstraint.h"
#include "ConstraintVisitor.h"

#include <simulation/collision_detection/narrow/Collision.h>

#include <simulation/ImpulseConstraintSolver.h>

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

void CollisionConstraint::initialize(double /*stepSize*/)
{
    mSumOfAllAppliedImpulses = Eigen::Vector::Zero();

    Eigen::Vector p1 = ImpulseConstraintSolver::calculateRelativePoint(
                mCollision.getSimulationObjectA(), mCollision.getPointA());
    Eigen::Vector p2 = ImpulseConstraintSolver::calculateRelativePoint(
                mCollision.getSimulationObjectB(), mCollision.getPointB());

    const Eigen::Vector& n = mCollision.getNormal();

    Eigen::Vector u1 = ImpulseConstraintSolver::calculateSpeed(
                mCollision.getSimulationObjectA(), p1, mCollision.getVertexIndexA());
    Eigen::Vector u2 = ImpulseConstraintSolver::calculateSpeed(
                mCollision.getSimulationObjectB(), p2, mCollision.getVertexIndexB());


    Eigen::Vector uRel = u1 - u2;

    mTargetUNormalRel = -mRestitution * uRel.dot(n) * n;

    mK = ImpulseConstraintSolver::calculateK(
                mCollision.getSimulationObjectA(), p1, mCollision.getVertexIndexA()) +
            ImpulseConstraintSolver::calculateK(
                mCollision.getSimulationObjectB(), p2, mCollision.getVertexIndexB());

    mImpulseFactor = 1 / (n.transpose() * mK * n);
}

bool CollisionConstraint::solve(double maxConstraintError)
{
    Eigen::Vector p1 = ImpulseConstraintSolver::calculateRelativePoint(
                mCollision.getSimulationObjectA(), mCollision.getPointA());
    Eigen::Vector p2 = ImpulseConstraintSolver::calculateRelativePoint(
                mCollision.getSimulationObjectB(), mCollision.getPointB());

    const Eigen::Vector& n = mCollision.getNormal();

    Eigen::Vector u1 = ImpulseConstraintSolver::calculateSpeed(
                mCollision.getSimulationObjectA(),
                p1,
                mCollision.getVertexIndexA());

    Eigen::Vector u2 = ImpulseConstraintSolver::calculateSpeed(
                mCollision.getSimulationObjectB(),
                p2,
                mCollision.getVertexIndexB());


    Eigen::Vector uRel = u1 - u2;

    Eigen::Vector uRelN = uRel.dot(n) * n;

    Eigen::Vector deltaUNormalRel = mTargetUNormalRel - uRelN;
    if (deltaUNormalRel.norm() < maxConstraintError)
    {
        return true;
    }
    Eigen::Vector impulse = mImpulseFactor * deltaUNormalRel;
    if (mCollision.getNormal().dot(mSumOfAllAppliedImpulses + impulse) < 0)
    {
        impulse = -mSumOfAllAppliedImpulses;
    }
    mSumOfAllAppliedImpulses += impulse;

    ImpulseConstraintSolver::applyImpulse(
                mCollision.getSimulationObjectA(), impulse, p1, mCollision.getVertexIndexA());
    ImpulseConstraintSolver::applyImpulse(
                mCollision.getSimulationObjectB(), -impulse, p2, mCollision.getVertexIndexB());


    // friction
    Eigen::Vector frictionImpulse;
    Eigen::Vector uRelT = uRel - uRelN;
    if (uRelT.norm() > 1e-8)
    {
        Eigen::Vector t = uRelT.normalized();
        Eigen::Vector frictionImpulseMax = - 1 / (t.transpose() * mK * t) * uRelT;

        if (mSumFrictionImpulses.norm() <= mCFrictionStatic * mSumOfAllAppliedImpulses.norm())
        {
            // static friction
            frictionImpulse = frictionImpulseMax;
        }
        else
        {
            // dynamic friction
            frictionImpulse = - mCFrictionDynamic * impulse.dot(n) * t;

            if (frictionImpulse.dot(t) > frictionImpulseMax.dot(t))
                frictionImpulse = frictionImpulseMax;

        }

        mSumFrictionImpulses += frictionImpulse;
        ImpulseConstraintSolver::applyImpulse(
                    mCollision.getSimulationObjectA(), frictionImpulse, p1, mCollision.getVertexIndexA());
        ImpulseConstraintSolver::applyImpulse(
                    mCollision.getSimulationObjectB(), -frictionImpulse, p2, mCollision.getVertexIndexB());
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
