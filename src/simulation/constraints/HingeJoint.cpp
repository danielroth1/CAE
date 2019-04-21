#include "BallJoint.h"
#include "DoubleAxisRotationalJoint.h"
#include "HingeJoint.h"

#include <simulation/rigid/RigidBody.h>

HingeJoint::HingeJoint(
        const std::shared_ptr<RigidBody>& rbA,
        const std::shared_ptr<RigidBody>& rbB,
        Eigen::Vector pointABS,
        Eigen::Vector pointBBS,
        Eigen::Vector axis1BS,
        Eigen::Vector axis2BS)
{
    SimulationPointRef refA(rbA.get(), rbA->getPolygon().get(), pointABS);
    SimulationPointRef refB(rbB.get(), rbB->getPolygon().get(), pointBBS);

    addConstraint(std::make_shared<BallJoint>(refA, refB));
    addConstraint(std::make_shared<DoubleAxisRotationalJoint>(rbA, rbB, axis1BS, axis2BS));
}
