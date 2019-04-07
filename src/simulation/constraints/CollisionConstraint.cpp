#include "CollisionConstraint.h"
#include "ConstraintVisitor.h"

#include <simulation/collision_detection/narrow/Collision.h>

#include <simulation/ImpulseConstraintSolver.h>

CollisionConstraint::CollisionConstraint(
        Collision& collision,
        double restitution)
    : mCollision(collision)
    , mRestitution(restitution)
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

void CollisionConstraint::initialize()
{
    mSumOfAllAppliedImpulses = Eigen::Vector::Zero();

    Eigen::Vector p1 = ImpulseConstraintSolver::calculateRelativePoint(
                mCollision.getSimulationObjectA(), mCollision.getPointA());
    Eigen::Vector p2 = ImpulseConstraintSolver::calculateRelativePoint(
                mCollision.getSimulationObjectB(), mCollision.getPointB());

    mTargetUNormalRel = -mRestitution *
            ImpulseConstraintSolver::calculateRelativeNormalSpeed(
                ImpulseConstraintSolver::calculateSpeed(
                    mCollision.getSimulationObjectA(), p1, mCollision.getVertexIndexA()),
                ImpulseConstraintSolver::calculateSpeed(
                    mCollision.getSimulationObjectB(), p2, mCollision.getVertexIndexB()),
                mCollision.getNormal());

    mImpulseFactor = 1 /
            (mCollision.getNormal().transpose() *
             (ImpulseConstraintSolver::calculateK(
                  mCollision.getSimulationObjectA(), p1, mCollision.getVertexIndexA()) +
              ImpulseConstraintSolver::calculateK(
                  mCollision.getSimulationObjectB(), p2, mCollision.getVertexIndexB()))
             * mCollision.getNormal());
}

bool CollisionConstraint::solve(double maxConstraintError)
{
    Eigen::Vector p1 = ImpulseConstraintSolver::calculateRelativePoint(
                mCollision.getSimulationObjectA(), mCollision.getPointA());
    Eigen::Vector p2 = ImpulseConstraintSolver::calculateRelativePoint(
                mCollision.getSimulationObjectB(), mCollision.getPointB());

    Eigen::Vector uRel =
            ImpulseConstraintSolver::calculateRelativeNormalSpeed(
                ImpulseConstraintSolver::calculateSpeed(
                    mCollision.getSimulationObjectA(), p1, mCollision.getVertexIndexA()),
                ImpulseConstraintSolver::calculateSpeed(
                    mCollision.getSimulationObjectB(), p2, mCollision.getVertexIndexB()),
                mCollision.getNormal());

    Eigen::Vector deltaUNormalRel = mTargetUNormalRel - uRel;
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
