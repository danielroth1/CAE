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

    Eigen::Vector p1 = ImpulseConstraintSolver::calculateRelativePoint(
                mCollision.getSimulationObjectA(), mCollision.getPointA());
    Eigen::Vector p2 = ImpulseConstraintSolver::calculateRelativePoint(
                mCollision.getSimulationObjectB(), mCollision.getPointB());

    const Eigen::Vector& n = mCollision.getNormal();

    Eigen::Vector u1 = ImpulseConstraintSolver::calculateSpeed(
                mCollision.getSimulationObjectA(), p1, mCollision.getVertexIndexA());
    Eigen::Vector u2 = ImpulseConstraintSolver::calculateSpeed(
                mCollision.getSimulationObjectB(), p2, mCollision.getVertexIndexB());


//    double positionCorrection = 0.2 * std::max(0.0, (mCollision.getPointA() - mCollision.getPointB()).dot(n) / stepSize);
    double positionCorrection = 0.0;//-std::max(0.0, (mCollision.getPointB() - mCollision.getPointA()).dot(n) / stepSize);
    if (positionCorrection > 1e-10)
        std::cout << "position correction = " << positionCorrection << "\n";
    Eigen::Vector uRel = u1 - u2;

    mTargetUNormalRel = (-mRestitution * uRel.dot(n) + positionCorrection) * n;

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
    }

    return false;
}

void CollisionConstraint::accept(ConstraintVisitor& cv)
{
    cv.visit(this);
}

bool CollisionConstraint::references(const std::shared_ptr<SimulationObject>& so)
{
    return so == mCollision.getSimulationObjectA() ||
            so == mCollision.getSimulationObjectB();
}
