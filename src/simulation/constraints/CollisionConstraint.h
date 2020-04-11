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
class CollisionConstraint : public Constraint
{
public:

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
            bool correctPositionError);

    virtual ~CollisionConstraint() override;

    const Collision& getCollision() const;

    const Eigen::Vector& getTargetUNormalRel() const;

    const Eigen::Vector& getSumOfAllAppliedImpulses() const;

    // Constraint interface
public:
    virtual void initialize(double stepSize) override;
    virtual bool solve(double maxConstraintError) override;
    virtual void accept(ConstraintVisitor& cv) override;
    virtual bool references(SimulationObject* so) override;

private:
    Collision mCollision;

    Eigen::Vector mTargetUNormalRel;
    Eigen::Vector mSumOfAllAppliedImpulses;
    Eigen::Matrix3d mK;
    Eigen::Vector mSumFrictionImpulses;
    double mImpulseFactor; // 1 / (n^T K_aa + K_bb n) * n
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

    // Temporary variables used in methods
    Eigen::Vector3d mPoint1;
    Eigen::Vector3d mPoint2;
    Eigen::Vector u1;
    Eigen::Vector u2;
    Eigen::Vector uRel;
    Eigen::Vector uRelN;
    Eigen::Vector deltaUNormalRel;
    Eigen::Vector impulse;
    Eigen::Vector frictionImpulse;
    Eigen::Vector uRelT;
    Eigen::Vector frictionImpulseMax;
};

#endif // COLLISIONCONSTRAINT_H
