#ifndef FIXEDROTATIONALJOINT_H
#define FIXEDROTATIONALJOINT_H

#include "Constraint.h"

#include <data_structures/DataStructures.h>

class RigidBody;

class FixedRotationalJoint : public Constraint
{
public:
    FixedRotationalJoint(
            const std::shared_ptr<RigidBody>& rbA,
            const std::shared_ptr<RigidBody>& rbB);

    // MechanicalProperty interface
public:
    virtual bool references(const std::shared_ptr<SimulationObject>& so);

    // Constraint interface
public:
    virtual void initialize(double stepSize);
    virtual bool solve(double maxConstraintError);
    virtual void accept(ConstraintVisitor& cv);

private:

    std::shared_ptr<RigidBody> mRbA;
    std::shared_ptr<RigidBody> mRbB;

    Eigen::Vector mTargetOmegaRel;
    Eigen::Matrix3d mImpulseFactor;
};

#endif // FIXEDROTATIONALJOINT_H
