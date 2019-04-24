#ifndef BALLJOINT_H
#define BALLJOINT_H

#include "Constraint.h"
#include "data_structures/DataStructures.h"

#include <simulation/references/SimulationPointRef.h>

class RigidBody;


// BallJoint Constraint:
// correction impulse calculation:
//
// u_rel = u_A - u_B                    <- nothing to do here
// targetURel:      0                   <- nothing to do here
// impulseFactor:   1 / (K_aa + K_bb)
//
// apply impulse:
//
// u_rel^current = u_A - u_B
// currentURel = u_rel^current
// delta_u = impulseFactor * (targetURel - currentURel)
//      = -impulseFactor * currentURel
//      = - 1 / (K_aa + K_bb) * currentURel
//
class BallJoint : public Constraint
{
public:
    BallJoint(SimulationPointRef pointA, SimulationPointRef pointB);

    const Eigen::Vector& getTargetURel() const;

    const Eigen::Vector& getSumOfAllAppliedImpulses() const;
    void setSumOfAllAppliedImpulses(const Eigen::Vector& impulses);

    // Constraint interface
public:
    virtual void initialize(double stepSize) override;
    virtual bool solve(double maxConstraintError) override;
    virtual void accept(ConstraintVisitor& cv) override;
    virtual bool references(const std::shared_ptr<SimulationObject>& so) override;

private:

    SimulationPointRef mPointA;
    SimulationPointRef mPointB;

    Eigen::Vector mTargetURel;
    Eigen::Vector mSumOfAllAppliedImpulses;
    Eigen::Matrix3d mImpulseFactor; // 1 / (K_aa + K_bb)

};

#endif // BALLJOINT_H
