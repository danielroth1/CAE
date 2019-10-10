#ifndef PLANEJOINT_H
#define PLANEJOINT_H

#include "Constraint.h"

#include <simulation/references/SimulationPointRef.h>


// A line joint removes one translational degree of freedome.
// One rigids point may only move on a line that is relative to a rigid
// asymetrical? how to define a line? SimpulationLineRef?

// In a plane joint two points are restircted in their movements in a way
// that they may only move on the plane
// s_A + \lambd_1 l_a a + \lambda_2 l_b.
//
// The plane is attached to simulation object A.
//
class PlaneJoint : public Constraint
{
public:
    PlaneJoint(
            SimulationPointRef pointA,
            SimulationPointRef pointB,
            Eigen::Vector lineDir1BS,
            Eigen::Vector lineDir2BS);

private:


    // MechanicalProperty interface
public:
    virtual bool references(const std::shared_ptr<SimulationObject>& so);

    // Constraint interface
public:
    virtual void initialize(double stepSize);
    virtual bool solve(double maxConstraintError);
    virtual void accept(ConstraintVisitor& cv);

private:

    // Sets mCurrentPlaneDir1 and mCurrentPlaneDir2.
    Eigen::Vector calculateLineDirection(const Eigen::Vector& lineBS);

    // Origin point of the line that is mounted to object A
    SimulationPointRef mPointA;

    // Point that can move freely on the line
    SimulationPointRef mPointB;

    Eigen::Vector mPoint1;
    Eigen::Vector mPoint2;

    // Direction of the line in body space coordinates.
    // If used on a deformable, the direction stays the same (for now).
    Eigen::Vector mLineDir1BS;
    Eigen::Vector mLineDir2BS;

    Eigen::Vector mCurrentAWS;
    Eigen::Vector mCurrentA;
    Eigen::Vector mCurrentLineDir1;
    Eigen::Vector mCurrentLineDir2;

    Eigen::Vector mTargetURel;
    Eigen::Matrix3d mImpulseFactor; // 1 / (K_aa + K_bb)
};

#endif // PLANEJOINT_H
