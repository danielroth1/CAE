#ifndef ROTATIONALMOTOR_H
#define ROTATIONALMOTOR_H

#include "Force.h"

#include <data_structures/DataStructures.h>

class RigidBody;

// A rotational motor applieds torques between two objects.
//
// It can be used to only rotate one object and ignore the other which might
// be usefull if the other object is a deformable for which no rotation
// information is avaoilable. Note though, that in these cases the torque
// wouldn't be the most physically accurate because it assumes the other
// object to have infinite mass.
//
class RotationalMotor : public Force
{
public:
    // A rotational motor that applies a torque on rb1 and the negative
    // torque on rb2. The torque is applied along the give axis that
    // is rotated with rb1.
    // \param rb1 - the first object that is rotated in clockwise direction.
    // \param rb2 - the second object that is rotated in counter clockwise
    //      direction. This can be null. If so, it is ignored and a torque
    //      is only applied to rb1.
    // \param axisBS - The axis around which the torque will be applied.
    // \param strength - The strength of the torque.
    RotationalMotor(
            const std::shared_ptr<RigidBody>& rb1,
            const std::shared_ptr<RigidBody>& rb2,
            const Eigen::Vector& axisBS,
            double strength = 1.0);

    void setStrength(double strength);
    double getStrength() const;

    // MechanicalProperty interface
public:
    virtual bool references(SimulationObject* so);

    // Force interface
public:
    virtual void applyForce();

private:
    std::shared_ptr<RigidBody> mRb1;
    std::shared_ptr<RigidBody> mRb2;
    Eigen::Vector mAxisBS;
    double mStrength;
};

#endif // ROTATIONALMOTOR_H
