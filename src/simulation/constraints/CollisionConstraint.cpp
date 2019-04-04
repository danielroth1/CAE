#include "CollisionConstraint.h"
#include "ConstraintVisitor.h"

#include <simulation/collision_detection/narrow/Collision.h>

CollisionConstraint::CollisionConstraint(
        Collision& collision,
        Eigen::Vector targetUNormalRel,
        Eigen::Vector sumOfAllAppliedImpulses,
        double impulseFactor)
    : mCollision(collision)
    , mTargetUNormalRel(targetUNormalRel)
    , mSumOfAllAppliedImpulses(sumOfAllAppliedImpulses)
    , mImpulseFactor(impulseFactor)
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

void CollisionConstraint::setSumOfAllAppliedImpulses(const Eigen::Vector& impulses)
{
    mSumOfAllAppliedImpulses = impulses;
}

double CollisionConstraint::getImpulseFactor()
{
    return mImpulseFactor;
}

void CollisionConstraint::setImpulseFactor(double impulseFactor)
{
    mImpulseFactor = impulseFactor;
}

void CollisionConstraint::accept(ConstraintVisitor& cv)
{
    cv.visit(this);
}

bool CollisionConstraint::references(Constraint* /*c*/)
{
    return false;
}

bool CollisionConstraint::references(SimulationObject* so)
{
    return so == mCollision.getSimulationObjectA() ||
            so == mCollision.getSimulationObjectB();
}
