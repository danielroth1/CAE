#include "RotationalMotor.h"

#include <simulation/rigid/RigidBody.h>

RotationalMotor::RotationalMotor(
        const std::shared_ptr<RigidBody>& rb1,
        const std::shared_ptr<RigidBody>& rb2,
        const Eigen::Vector& axisBS,
        double strength)
    : mRb1(rb1)
    , mRb2(rb2)
    , mAxisBS(axisBS)
    , mStrength(strength)
{
    mAxisBS.normalize();
}

void RotationalMotor::setStrength(double strength)
{
    mStrength = strength;
}

double RotationalMotor::getStrength() const
{
    return mStrength;
}

bool RotationalMotor::references(SimulationObject* so)
{
    return mRb1.get() == so || mRb2.get() == so;
}

void RotationalMotor::applyForce()
{
    Eigen::Vector axisWS = mRb1->getOrientation().toRotationMatrix() * mAxisBS;

    // The torque is represented in axis-angle coordinates
    // The norm is the orientation axis.
    // The magnitude is the strength.
    Eigen::Vector torque = mStrength * axisWS;
    mRb1->applyTorque(torque);
    mRb2->applyTorque(-torque);
}
