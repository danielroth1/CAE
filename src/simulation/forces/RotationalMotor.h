#ifndef ROTATIONALMOTOR_H
#define ROTATIONALMOTOR_H

#include "Force.h"

#include <data_structures/DataStructures.h>

class RigidBody;

class RotationalMotor : public Force
{
public:
    // A rotational motor that applies a torque on rb1 and the negative
    // torque on rb2. The torque is applied along the give axis that
    // is rotated with rb1.
    RotationalMotor(
            const std::shared_ptr<RigidBody>& rb1,
            const std::shared_ptr<RigidBody>& rb2,
            const Eigen::Vector& axisBS,
            double strength = 1.0);

    void setStrength(double strength);
    double getStrength() const;

    // MechanicalProperty interface
public:
    virtual bool references(const std::shared_ptr<SimulationObject>& so);

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
