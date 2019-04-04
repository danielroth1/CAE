#ifndef BALLJOINT_H
#define BALLJOINT_H

#include "Constraint.h"
#include "data_structures/DataStructures.h"

#include <simulation/references/SimulationPointRef.h>

class RigidBody;

class BallJoint : public Constraint
{
public:
    BallJoint(SimulationPointRef pointA, SimulationPointRef pointB);

    const Eigen::Vector& getTargetURel() const;

    const Eigen::Vector& getSumOfAllAppliedImpulses() const;
    void setSumOfAllAppliedImpulses(const Eigen::Vector& impulses);

    double getImpulseFactor();
    void setImpulseFactor(double impulseFactor);

    // Constraint interface
public:
    virtual void accept(ConstraintVisitor& cv);
    virtual bool references(Constraint* c) = 0;
    virtual bool references(SimulationObject* so) = 0;

private:

    SimulationPointRef mPointA;
    SimulationPointRef mPointB;

    Eigen::Vector mTargetURel;
    Eigen::Vector mSumOfAllAppliedImpulses;
    double mImpulseFactor; // 1 / (n^T K_aa + K_bb n) * n

};

#endif // BALLJOINT_H
