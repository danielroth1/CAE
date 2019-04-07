#ifndef COLLISIONCONSTRAINT_H
#define COLLISIONCONSTRAINT_H

#include "data_structures/DataStructures.h"

#include "Constraint.h"

class Collision;

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
    CollisionConstraint(Collision& collision, double restitution);

    virtual ~CollisionConstraint() override;

    Collision& getCollision();

    const Eigen::Vector& getTargetUNormalRel() const;

    const Eigen::Vector& getSumOfAllAppliedImpulses() const;

    // Constraint interface
public:
    virtual void initialize() override;
    virtual bool solve(double maxConstraintError) override;
    virtual void accept(ConstraintVisitor& cv) override;
    virtual bool references(SimulationObject* so) override;

private:
    Collision& mCollision;
    Eigen::Vector mTargetUNormalRel;
    Eigen::Vector mSumOfAllAppliedImpulses;
    double mImpulseFactor; // 1 / (n^T K_aa + K_bb n) * n
    double mRestitution;
};

#endif // COLLISIONCONSTRAINT_H
