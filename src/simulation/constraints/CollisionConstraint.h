#ifndef COLLISIONCONSTRAINT_H
#define COLLISIONCONSTRAINT_H

#include "data_structures/DataStructures.h"

#include "Constraint.h"

#include <simulation/collision_detection/narrow/Collision.h>

// Implements collision and friction based on
// "Constraint-based collision and contact handling using impulses"
// by Bender et al. Another implementation can be found in Benders library
// "IBDS".
//
// Collision Constraint:
// Correction impulse calculation:
//
// u_rel = u_A - u_B
// Target velocity: targetUNormalRel = -res * u_rel * n
// impulseFactor:   1 / (n^t * (K_aa + K_bb) * n)
//
// applying impulse:
//
// u_rel^current = u_A - u_B
// currentUNormalRel = u_rel^current * n
// delta_u_normal = impulseFactor * (targetUNormalRel - currentUNormalRel)
//
// Friction: Supports two types of frictions. Switch between them by setting
//   the defines in the body of this class.
//   For details on the methods, see documentation of
//   - solveTwoTangentFrictionConstraint1
//   - solveTwoTangentFrictionConstraint2
//   - solveSingleTangentFrictionConstraint
class CollisionConstraint : public Constraint
{
public:

    CollisionConstraint();

    //
    // ContactMargin:
    // The idea is to lets constraints not correct the whole contact distance
    // but substract a value contactMargin so they only correct:
    // collisionMargin - contactMargin
    // Now contacts remain valid because they keep within the distance of
    // [collisionMargin, collisionMargin - contactMargin]
    //
    // \param restitution - [0 - 1] bounciness: objects are bouncing off by
    //      this factor (velocity in normal direction is reflected). A
    //      restitution of 0 prevents any bouncing. 1 means input velocity is
    //      equal to output velocity making objects bounce infinetly. Values
    //      smaller than 1 can be considered as damping and take energy out
    //      of the system.
    // \param positionCorrectionFactor - this constraint corrects the position
    //      error times this factor (only if correctPositionError is true).
    // \param collisionMargin - if the minimum distance between two objects
    //      is smaller than this value, the constraint is valid. The constraint
    //      tries to preserve this distance by applying a position correction.
    // \param contactMargin - Margin at which no position correction is applied
    //      anymore to ensure that the collision remains during the next
    //      simulation step. Those collising are called "contacts".
    //      The contact distance is collisionMargin - contactMargin. Typically
    //      the contact margin should be a lot smaller than the collisionMargin.
    //      If its too high, object that usually lie parallel on top of each
    //      other can be not parallel anymore and slide off of each other.
    // \param correctPositionError - if true, the position error is corrected.
    //      If false, collisionMargin and contactMargin are not used.
    CollisionConstraint(
            const Collision& collision,
            double restitution,
            double positionCorrectionFactor,
            double collisionMargin,
            double contactMargin,
            bool correctPositionError,
            bool applyWarmStarting);

    CollisionConstraint(
            const Collision& collision,
            double restitution,
            double positionCorrectionFactor,
            double collisionMargin,
            double contactMargin,
            bool correctPositionError,
            bool applyWarmStarting,
            const Eigen::Vector3d& warmStartingCollisionImpulses,
            const Eigen::Vector2d& warmStartingFrictionImpulses);

    virtual ~CollisionConstraint() override;

    void setCollision(const Collision& collision);
    const Collision& getCollision() const;

    const Eigen::Vector& getTargetUNormalRel() const;

    void setSumCollisionImpulses(const Eigen::Vector3d& sumOfAllAppliedImpulses);
    const Eigen::Vector& getSumCollisionImpulses() const;

    const Eigen::Vector& getSumCollisionImpulsesOld() const;

    void setSumFrictionImpulses(const Eigen::Vector2d& sumFrictionImpulses);
    const Eigen::Vector2d& getSumFrictionImpulses() const;

    const Eigen::Vector3d& getTangent1() const
    {
        return mFrT1;
    }

    const Eigen::Vector3d& getTangent2() const
    {
        return mFrT2;
    }

    void setReuseCount(double reuseCount)
    {
        mReuseCount = reuseCount;
    }

    int getReuseCount() const
    {
        return mReuseCount;
    }

    // Constraint interface
public:
    virtual void initialize(double stepSize) override;
    virtual void applyWarmStarting() override;
    virtual bool solve(double maxConstraintError) override;
    virtual void accept(ConstraintVisitor& cv) override;
    virtual bool references(SimulationObject* so) override;

private:

    // Solve the friction constraint by using two friction tangents. Using
    // two tangents is a lot more robust than one because velocity changes
    // within the friction plane can be considered instead of only within
    // a single line.
    // These constraint can be applied right after collision constraints in
    // the same solving cycle (in contrast to single tangent constraints that
    // should be applied only once alle collision constraints were).
    void solveTwoTangentFrictionConstraint1();

    // Slightly different formulation than solveTwoTangentFrictionConstraint2()
    // which gives minor different results. Not sure which one is the correct
    // version.
    void solveTwoTangentFrictionConstraint2();

    // Solve the friction constraint using a single friction tangent.
    // There are some issues with this implementation:
    // - it really only works if the friction constraints are applied
    //   after all collision constraints were applied. This means that
    //   friction constraints can violate other constraints at the end again.
    // - this implementation doesn't work with warm starting
    void solveSingleTangentFrictionConstraint(const Eigen::Vector3d& impulse);

    Collision mCollision;

    Eigen::Vector mTargetUNormalRel;
    Eigen::Vector mSumCollisionImpulses;
    Eigen::Vector mSumCollisionImpulsesOld;
    Eigen::Vector mSumFrictionImpulses;
    Eigen::Matrix3d mK;
    double mImpulseFactor; // 1 / (n^T K_aa + K_bb n)

    // friction
    Eigen::Vector3d mFrictionTangent;

    double mFrictionImpulseMass; // 1 / (t^T K_aa + K_bb t)

    double mRestitution;
    double mCFrictionDynamic;
    double mCFrictionStatic;
    double mPositionCorrectionFactor;
    double mCollisionMargin;
    // Margin at which no position correction is applied anymore to ensure that
    // the collision remains during the next simulation step. Those collising
    // are called "contacts".
    double mContactMargin;
    bool mSticking;
    bool mCorrectPositionError;

    bool mApplyFriction;

    Eigen::Vector3d mFrT1; // friction tangent 1
    Eigen::Vector3d mFrT2; // friction tangent 2
    Eigen::Matrix2d mFrInvMass; // friction mass
    Eigen::Vector2d mFrSum;

    double m_pf_total;
    double m_pf_actual;

    // Temporary variables used in methods
    Eigen::Vector3d mPoint1;
    Eigen::Vector3d mPoint2;

    bool mWarmStarting;

    int mReuseCount; // counts how often this constraint has been reused
};

#endif // COLLISIONCONSTRAINT_H
