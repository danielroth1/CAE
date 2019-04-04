#ifndef COLLISIONCONSTRAINT_H
#define COLLISIONCONSTRAINT_H

#include "data_structures/DataStructures.h"

#include "Constraint.h"

class Collision;

class CollisionConstraint : public Constraint
{
public:
    CollisionConstraint(Collision& collision,
                        Eigen::Vector targetUNormalRel,
                        Eigen::Vector sumOfAllAppliedImpulses,
                        double impulseFactor);

    virtual ~CollisionConstraint();

    Collision& getCollision();

    const Eigen::Vector& getTargetUNormalRel() const;

    const Eigen::Vector& getSumOfAllAppliedImpulses() const;
    void setSumOfAllAppliedImpulses(const Eigen::Vector& impulses);

    double getImpulseFactor();
    void setImpulseFactor(double impulseFactor);

    // Constraint interface
public:
    virtual void accept(ConstraintVisitor& cv);
    virtual bool references(Constraint* c);
    virtual bool references(SimulationObject* so);

private:
    Collision& mCollision;
    Eigen::Vector mTargetUNormalRel;
    Eigen::Vector mSumOfAllAppliedImpulses;
    double mImpulseFactor; // 1 / (n^T K_aa + K_bb n) * n

};

#endif // COLLISIONCONSTRAINT_H
