#ifndef AXISROTATIONALJOINT_H
#define AXISROTATIONALJOINT_H

#include "Constraint.h"

#include <data_structures/DataStructures.h>

#include <simulation/references/SimulationPointRef.h>

class AxisRotationalJoint : public Constraint
{
public:
    AxisRotationalJoint(
            const std::shared_ptr<RigidBody>& rbA,
            const std::shared_ptr<RigidBody>& rbB,
            Eigen::Vector axisBS);

    // MechanicalProperty interface
public:
    virtual bool references(SimulationObject* so);

    // Constraint interface
public:
    virtual void initialize(double stepSize);
    virtual bool solve(double maxConstraintError);
    virtual void accept(ConstraintVisitor& cv);

private:

    std::shared_ptr<RigidBody> mRbA;
    std::shared_ptr<RigidBody> mRbB;
    Eigen::Vector mAxisBS;

    Eigen::Matrix<double, 2, 3> mProjMatrix;
    Eigen::Vector2d mTargetOmegaRel;
    Eigen::Matrix2d mImpulseFactor;
};

#endif // AXISROTATIONALJOINT_H
