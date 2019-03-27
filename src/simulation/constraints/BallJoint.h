#ifndef BALLJOINT_H
#define BALLJOINT_H

#include "Constraint.h"
#include "data_structures/DataStructures.h"

class RigidBody;

class BallJoint : public Constraint
{
public:
    BallJoint(RigidBody* rb1, RigidBody* rb2);

    double calculateValue();

    Eigen::Vector calculateGradient();


    // Constraint interface
public:
    virtual void accept(ConstraintVisitor& cv);

};

#endif // BALLJOINT_H
