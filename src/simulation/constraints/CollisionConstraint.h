#ifndef COLLISIONCONSTRAINT_H
#define COLLISIONCONSTRAINT_H

#include "data_structures/DataStructures.h"

#include "Constraint.h"

class Collision;

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
    CollisionConstraint(
            Collision& collision,
            double restitution,
            double cFrictionDynamic,
            double cFrictionStatic);

    virtual ~CollisionConstraint() override;

    Collision& getCollision();

    const Eigen::Vector& getTargetUNormalRel() const;

    const Eigen::Vector& getSumOfAllAppliedImpulses() const;

    // Constraint interface
public:
    virtual void initialize(double stepSize) override;
    virtual bool solve(double maxConstraintError) override;
    virtual void accept(ConstraintVisitor& cv) override;
    virtual bool references(SimulationObject* so) override;

private:
    Collision& mCollision;

    Eigen::Vector mTargetUNormalRel;
    Eigen::Vector mSumOfAllAppliedImpulses;
    Eigen::Matrix3d mK;
    Eigen::Vector mSumFrictionImpulses;
    double mImpulseFactor; // 1 / (n^T K_aa + K_bb n) * n
    double mRestitution;
    double mCFrictionDynamic;
    double mCFrictionStatic;
    bool mSticking;

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
