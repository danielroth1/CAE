#include "CollisionConstraint.h"
#include "ConstraintVisitor.h"

#include <simulation/collision_detection/narrow/Collision.h>

#include <simulation/ImpulseConstraintSolver.h>

#include <iostream>

#include <simulation/fem/FEMObject.h>

#include <math/MathUtils.h>

// See class documentation for details on these defines.
#define FRICTION_TWO_TANGENT_1
//#define FRICTION_TWO_TANGENT_2
//#define FRICTION_SINGLE_TANGENT

CollisionConstraint::CollisionConstraint()
{

}

CollisionConstraint::CollisionConstraint(
        const Collision& collision,
        double restitution,
        double positionCorrectionFactor,
        double collisionMargin,
        double contactMargin,
        bool correctPositionError,
        bool applyWarmStarting)
    : CollisionConstraint(
          collision,
          restitution,
          positionCorrectionFactor,
          collisionMargin,
          contactMargin,
          correctPositionError,
          applyWarmStarting,
          Eigen::Vector3d::Zero(),
          Eigen::Vector2d::Zero())
{
}

CollisionConstraint::CollisionConstraint(
        const Collision& collision,
        double restitution,
        double positionCorrectionFactor,
        double collisionMargin,
        double contactMargin,
        bool correctPositionError,
        bool applyWarmStarting,
        const Vector3d& warmStartingCollisionImpulses,
        const Vector2d& warmStartingFrictionImpulses)
    : mCollision(collision)
    , mRestitution(restitution)
    , mPositionCorrectionFactor(positionCorrectionFactor)
    , mCollisionMargin(collisionMargin)
    , mContactMargin(contactMargin)
    , mCorrectPositionError(correctPositionError)
    , mWarmStarting(applyWarmStarting)
{
    mCFrictionStatic = std::sqrt(
                (collision.getSimulationObjectA()->getFrictionStatic() *
                 collision.getSimulationObjectB()->getFrictionStatic()));

    mCFrictionDynamic = std::sqrt(
                (collision.getSimulationObjectA()->getFrictionDynamic() *
                 collision.getSimulationObjectB()->getFrictionDynamic()));

    mSumCollisionImpulses = warmStartingCollisionImpulses;
    mFrSum = warmStartingFrictionImpulses;
    m_pf_total = 0;
    m_pf_actual = 0;

    mReuseCount = 0;

}

CollisionConstraint::~CollisionConstraint()
{

}

void CollisionConstraint::setCollision(const Collision& collision)
{
   mCollision = collision;
}

const Collision& CollisionConstraint::getCollision() const
{
    return mCollision;
}

const Eigen::Vector& CollisionConstraint::getTargetUNormalRel() const
{
    return mTargetUNormalRel;
}

void CollisionConstraint::setSumCollisionImpulses(const Vector3d& sumCollisionImpulses)
{
    mSumCollisionImpulses = sumCollisionImpulses;
}

const Eigen::Vector& CollisionConstraint::getSumCollisionImpulses() const
{
    return mSumCollisionImpulses;
}

const Vector& CollisionConstraint::getSumCollisionImpulsesOld() const
{
    return mSumCollisionImpulsesOld;
}

void CollisionConstraint::setSumFrictionImpulses(const Vector2d& sumFrictionImpulses)
{
//    m_pf_total = sumFrictionImpulses.norm();
//    m_pf_actual = sumFrictionImpulses.norm();
    mFrSum = sumFrictionImpulses;
}

const Eigen::Vector2d& CollisionConstraint::getSumFrictionImpulses() const
{
    return mFrSum;
//    return mSumFrictionImpulses;
}

void CollisionConstraint::initialize(double stepSize)
{
    mSticking = false;

    mPoint1 = ImpulseConstraintSolver::calculateRelativePoint(
                mCollision.getSimulationObjectA(), mCollision.getPointA());
    mPoint2 = ImpulseConstraintSolver::calculateRelativePoint(
                mCollision.getSimulationObjectB(), mCollision.getPointB());

    const Eigen::Vector& n = mCollision.getNormal();

    Eigen::Vector3d u1 = ImpulseConstraintSolver::calculateSpeed(
                mCollision.getSimulationObjectA(), mPoint1,
                mCollision.getBarycentricCoordiantesA(), mCollision.getElementIdA());
    Eigen::Vector3d u2 = ImpulseConstraintSolver::calculateSpeed(
                mCollision.getSimulationObjectB(), mPoint2,
                mCollision.getBarycentricCoordiantesB(), mCollision.getElementIdB());

    Eigen::Vector3d uRel = u1 - u2;

    if (mCorrectPositionError /*&& !mCollision.isInside()*/)
    {
        double posError = (mCollision.getPointA() - mCollision.getPointB()).dot(n);
        posError = std::min(posError - mCollisionMargin + mContactMargin, 0.0);
        double positionCorrection = -mPositionCorrectionFactor * posError / stepSize;
        mTargetUNormalRel = (-mRestitution * uRel.dot(n) + positionCorrection) * n;
    }
    else
    {
        mTargetUNormalRel = (-mRestitution * uRel.dot(n)) * n;
    }

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

    mApplyFriction = false;
    // friction
    if (!mCollision.isInside() &&
        (mCFrictionStatic > 1e-15 ||
        mCFrictionDynamic > 1e-15))
    {
        uRel = u1 - u2;
        Eigen::Vector3d uRelN = uRel.dot(mCollision.getNormal()) * mCollision.getNormal();
        Eigen::Vector3d uRelT = uRel - uRelN;

        if (uRelT.norm() > 1e-8)
        {
            mFrictionTangent = uRelT.normalized();
            mFrictionImpulseMass =
                    - 1 / (mFrictionTangent.transpose() * mK * mFrictionTangent);
            mApplyFriction = true;

            mFrT1 = MathUtils::perp(n);
            mFrT2 = n.cross(mFrT1);
            double k11 = mFrT1.transpose() * mK * mFrT1;
            double k12 = mFrT1.transpose() * mK * mFrT2;
            double k22 = mFrT2.transpose() * mK * mFrT2;
            mFrInvMass << k11, k12,
                    k12, k22;
            mFrInvMass = -mFrInvMass.inverse();
        }
    }


}

void CollisionConstraint::applyWarmStarting()
{
    if (mWarmStarting)
    {
        const Eigen::Vector& n = mCollision.getNormal();

        Eigen::Vector3d impulse = mSumCollisionImpulses.dot(n) * n;
        mSumCollisionImpulses = impulse;
        ImpulseConstraintSolver::applyImpulse(
                    mCollision.getSimulationObjectA(), impulse,
                    mPoint1, mCollision.getBarycentricCoordiantesA(),
                    mCollision.getElementIdA());

        ImpulseConstraintSolver::applyImpulse(
                    mCollision.getSimulationObjectB(), -impulse,
                    mPoint2, mCollision.getBarycentricCoordiantesB(),
                    mCollision.getElementIdB());

        if (mApplyFriction &&
                !mCollision.isInside() &&
                (mCFrictionStatic > 1e-15 ||
                 mCFrictionDynamic > 1e-15))
        {

            Eigen::Vector3d pf3 = mFrSum(0) * mFrT1 + mFrSum(1) * mFrT2;

            ImpulseConstraintSolver::applyImpulse(
                        mCollision.getSimulationObjectA(), pf3,
                        mPoint1, mCollision.getBarycentricCoordiantesA(),
                        mCollision.getElementIdA());

            ImpulseConstraintSolver::applyImpulse(
                        mCollision.getSimulationObjectB(), -pf3,
                        mPoint2, mCollision.getBarycentricCoordiantesB(),
                        mCollision.getElementIdB());
        }

    }
}

bool CollisionConstraint::solve(double maxConstraintError)
{
    const Eigen::Vector& n = mCollision.getNormal();

    Eigen::Vector3d u1 = ImpulseConstraintSolver::calculateSpeed(
                mCollision.getSimulationObjectA(), mPoint1,
                mCollision.getBarycentricCoordiantesA(), mCollision.getElementIdA());
    Eigen::Vector3d u2 = ImpulseConstraintSolver::calculateSpeed(
                mCollision.getSimulationObjectB(), mPoint2,
                mCollision.getBarycentricCoordiantesB(), mCollision.getElementIdB());

    Eigen::Vector3d uRel = u1 - u2;

    Eigen::Vector3d uRelN = uRel.dot(n) * n;

    Eigen::Vector3d deltaUNormalRel = mTargetUNormalRel - uRelN;

    bool appliedCollisionImpulse;
    if (deltaUNormalRel.squaredNorm() < maxConstraintError * maxConstraintError)
    {
        appliedCollisionImpulse = false;
    }
    else
    {
        appliedCollisionImpulse = true;
        Eigen::Vector3d impulse = mImpulseFactor * deltaUNormalRel;
        if (mCollision.getNormal().dot(mSumCollisionImpulses + impulse) < 0)
        {
            impulse = -mSumCollisionImpulses;
        }
        mSumCollisionImpulses += impulse;

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

    // bounce friction
    if (mApplyFriction)
    {
        if (appliedCollisionImpulse &&
                !mCollision.isInside() &&
                mCFrictionDynamic > 1e-15)
        {
#if defined(FRICTION_TWO_TANGENT_1)
            solveTwoTangentFrictionConstraint1();
#elif defined(FRICTION_TWO_TANGENT_2)
            solveTwoTangentFrictionConstraint2();
#elif defined(FRICTION_SINGLE_TANGENT)
            if (mCFrictionStatic > 1e-15)
            {
                solveSingleTangentFrictionConstraint(impulse);
            }
#endif
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

void CollisionConstraint::solveTwoTangentFrictionConstraint1()
{
    const Eigen::Vector& n = mCollision.getNormal();

    // recalculate uRel after the collision impulse was applied
    Eigen::Vector3d u1 = ImpulseConstraintSolver::calculateSpeed(
                mCollision.getSimulationObjectA(), mPoint1,
                mCollision.getBarycentricCoordiantesA(), mCollision.getElementIdA());
    Eigen::Vector3d u2 = ImpulseConstraintSolver::calculateSpeed(
                mCollision.getSimulationObjectB(), mPoint2,
                mCollision.getBarycentricCoordiantesB(), mCollision.getElementIdB());

    Eigen::Vector3d uRel = u1 - u2;

    Eigen::Vector2d uRelT;
    uRelT << uRel.dot(mFrT1),
            uRel.dot(mFrT2);

    double pfFactor = -mCFrictionDynamic * mSumCollisionImpulses.dot(n);// - mFrSum.norm();

    Eigen::Vector2d pfMaxFactor = mFrInvMass * uRelT;

    Eigen::Vector2d pf;
    Eigen::Vector2d pfMaxPlusOld = pfMaxFactor + mFrSum;
    if (pfFactor * pfFactor < pfMaxPlusOld.dot(pfMaxPlusOld))
    {
        pf = -pfFactor * pfMaxPlusOld.normalized() - mFrSum;
    }
    else
    {
        pf = pfMaxFactor;
    }

    mFrSum += pf;

    Eigen::Vector3d pf3 = pf(0) * mFrT1 + pf(1) * mFrT2;
    ImpulseConstraintSolver::applyImpulse(
                mCollision.getSimulationObjectA(), pf3,
                mPoint1, mCollision.getBarycentricCoordiantesA(),
                mCollision.getElementIdA());

    ImpulseConstraintSolver::applyImpulse(
                mCollision.getSimulationObjectB(), -pf3,
                mPoint2, mCollision.getBarycentricCoordiantesB(),
                mCollision.getElementIdB());
}

void CollisionConstraint::solveTwoTangentFrictionConstraint2()
{
    const Eigen::Vector& n = mCollision.getNormal();

    // recalculate uRel after the collision impulse was applied
    Eigen::Vector3d u1 = ImpulseConstraintSolver::calculateSpeed(
                mCollision.getSimulationObjectA(), mPoint1,
                mCollision.getBarycentricCoordiantesA(), mCollision.getElementIdA());
    Eigen::Vector3d u2 = ImpulseConstraintSolver::calculateSpeed(
                mCollision.getSimulationObjectB(), mPoint2,
                mCollision.getBarycentricCoordiantesB(), mCollision.getElementIdB());

    Eigen::Vector3d uRel = u1 - u2;

    Eigen::Vector2d uRelT;
    uRelT << uRel.dot(mFrT1),
            uRel.dot(mFrT2);

    Eigen::Vector2d impulse = mFrInvMass * uRelT;
    Eigen::Vector2d oldImpulse = mFrSum;
    mFrSum += impulse;

    double maxImpulse = mCFrictionDynamic * mSumCollisionImpulses.dot(n);

    if (mFrSum.dot(mFrSum) > maxImpulse * maxImpulse)
    {
        mFrSum.normalize();
        mFrSum *= maxImpulse;
    }
    impulse = mFrSum - oldImpulse;

    Eigen::Vector3d pf3 = impulse(0) * mFrT1 + impulse(1) * mFrT2;
    ImpulseConstraintSolver::applyImpulse(
                mCollision.getSimulationObjectA(), pf3,
                mPoint1, mCollision.getBarycentricCoordiantesA(),
                mCollision.getElementIdA());

    ImpulseConstraintSolver::applyImpulse(
                mCollision.getSimulationObjectB(), -pf3,
                mPoint2, mCollision.getBarycentricCoordiantesB(),
                mCollision.getElementIdB());
}

void CollisionConstraint::solveSingleTangentFrictionConstraint(
        const Eigen::Vector3d& impulse)
{
    const Eigen::Vector3d n = mCollision.getNormal();

    // recalculate uRel after the collision impulse was applied
    Eigen::Vector3d u1 = ImpulseConstraintSolver::calculateSpeed(
                mCollision.getSimulationObjectA(), mPoint1,
                mCollision.getBarycentricCoordiantesA(), mCollision.getElementIdA());
    Eigen::Vector3d u2 = ImpulseConstraintSolver::calculateSpeed(
                mCollision.getSimulationObjectB(), mPoint2,
                mCollision.getBarycentricCoordiantesB(), mCollision.getElementIdB());

    Eigen::Vector3d uRel = u1 - u2;


//        uRelT = uRel.dot(tangent) * tangent;
//        uRelT = uRel.dot(mFrictionTangent) * mFrictionTangent;

    Eigen::Vector3d uRelN = uRel.dot(n) * n;
    Eigen::Vector3d uRelT = uRel - uRelN;
    Eigen::Vector3d tangent = uRelT.normalized();

    mFrictionImpulseMass =
            - 1 / (tangent.transpose() * mK * tangent);

    if (uRelT.norm() > 1e-8)
    {
////            Eigen::Vector tangent = uRelT.normalized();
//            frictionImpulseMax = - 1 / (tangent.transpose() * mK * tangent) * uRelT;

        // This is the equation from the paper.
        double pfFactor = -mCFrictionDynamic * impulse.dot(n);
        m_pf_total += pfFactor;

//            pfFactor = m_pf_total - m_pf_actual;

//            Eigen::Vector3d tangent = uRelT.normalized();
        Eigen::Vector3d pfMax = mFrictionImpulseMass * uRelT;
//            double pfMaxFactor = pfMax.dot(mFrictionTangent);
        double pfMaxFactor = pfMax.dot(tangent);
        double value = std::max(m_pf_actual + pfFactor, -(m_pf_actual + pfFactor));

        if (mSticking && value <= mCFrictionStatic * mSumCollisionImpulses.norm())
            mSticking = true;
        else
            mSticking = false;

        if (mSticking || pfFactor < pfMaxFactor)// || pfFactor > -pfMaxFactor)
        {
//                std::cout << m_pf_actual + pfFactor << " < " << mCFrictionStatic * mSumCollisionImpulses.norm() << " = "
//                          << (m_pf_actual + pfFactor <= mCFrictionStatic * mSumCollisionImpulses.norm()) << "\n";
            pfFactor = pfMaxFactor;
            mSticking = true;
        }

//            if (pfFactor * pfFactor > maxConstraintError * maxConstraintError)
        {
            m_pf_actual += pfFactor;

//                Eigen::Vector3d pf = pfFactor * mFrictionTangent;
            Eigen::Vector3d pf = pfFactor * tangent;

            ImpulseConstraintSolver::applyImpulse(
                        mCollision.getSimulationObjectA(), pf,
                        mPoint1, mCollision.getBarycentricCoordiantesA(),
                        mCollision.getElementIdA());

            ImpulseConstraintSolver::applyImpulse(
                        mCollision.getSimulationObjectB(), -pf,
                        mPoint2, mCollision.getBarycentricCoordiantesB(),
                        mCollision.getElementIdB());
        }

    }
}
