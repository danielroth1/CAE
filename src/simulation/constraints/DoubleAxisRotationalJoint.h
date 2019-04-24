#ifndef DOUBLEAXISROTATIONALJOINT_H
#define DOUBLEAXISROTATIONALJOINT_H

#include "Constraint.h"

#include <data_structures/DataStructures.h>

#include <simulation/references/SimulationPointRef.h>

class DoubleAxisRotationalJoint : public Constraint
{
public:
    DoubleAxisRotationalJoint(
            const std::shared_ptr<RigidBody>& rbA,
            const std::shared_ptr<RigidBody>& rbB,
            Eigen::Vector axis1BS,
            Eigen::Vector axis2BS);

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
    Eigen::Vector mAxis1BS;
    Eigen::Vector mAxis2BS;

    Eigen::Matrix<double, 1, 3> mProjMatrix;
    Eigen::Vector mProjMatrixT;
    double mTargetOmegaRel;
    double mImpulseFactor;
};

#endif // DOUBLEAXISROTATIONALJOINT_H
