#ifndef DISTANCEJOINT_H
#define DISTANCEJOINT_H

#include "Constraint.h"
#include "data_structures/DataStructures.h"

#include <simulation/references/SimulationPointRef.h>

class RigidBody;


// DistanceJoint Constraint:
// constraints the distance between two points:
// | x_a - x_b | = d
// If d == 0, this becomes a BallJoint. BallJoint should still be preferred
// becaues its more optimized.
//
class DistanceJoint : public Constraint
{
public:
    DistanceJoint(SimulationPointRef pointA, SimulationPointRef pointB, double distance = 0.0);

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

    Eigen::Vector mPoint1;
    Eigen::Vector mPoint2;

    Eigen::Vector mTargetURel;
    Eigen::Vector mSumOfAllAppliedImpulses;
    Eigen::Matrix3d mImpulseFactor; // 1 / (K_aa + K_bb)

    double mDistance;
    double mStepSize;
};

#endif // DISTANCEJOINT_H
