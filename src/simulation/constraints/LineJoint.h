#ifndef LINEJOINT_H
#define LINEJOINT_H

#include "Constraint.h"

#include <simulation/references/SimulationPointRef.h>


// A line joint removes one translational degree of freedome.
// One rigids point may only move on a line that is relative to a rigid
// asymetrical? how to define a line? SimpulationLineRef?

// In a line joint two points are restircted in their movements in a way
// that they may only move on the line
// A + \lambda a.
// A and a are arbitrary parameters that can be updated
//
// There are multiple ways to calculate the correction impulse
// One is to project deltaURel to point only in direction of B - A.
// The other is to use benders way of projecting the correction
// impulse.
class LineJoint : public Constraint
{
public:
    LineJoint(SimulationPointRef pointA,
              SimulationPointRef pointB,
              Eigen::Vector lineDirectionBS);

private:


    // MechanicalProperty interface
public:
    virtual bool references(SimulationObject* so);

    // Constraint interface
public:
    virtual void initialize(double stepSize);
    virtual bool solve(double maxConstraintError);
    virtual void accept(ConstraintVisitor& cv);

private:

    Eigen::Vector calculateLineDirection();

    // Origin point of the line that is mounted to object A
    SimulationPointRef mPointARef;

    // Point that can move freely on the line
    SimulationPointRef mPointBRef;

    Eigen::Vector mPointBWS;
    Eigen::Vector mPointBBS;

    // Direction of the line in body space coordinates.
    // If used on a deformable, the direction stays the same (for now).
    Eigen::Vector mLineDirectionBS;

    Eigen::Vector mPointAWS;
    Eigen::Vector mPointABS;
    Eigen::Vector mCurrentLineDir;

    Eigen::Vector mTargetURel;
    Eigen::Matrix3d mImpulseFactor; // 1 / (K_aa + K_bb)
};

#endif // LINEJOINT_H
