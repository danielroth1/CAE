#include "FixedRotationalJointDemo.h"

#include <ApplicationControl.h>

#include <simulation/rigid/RigidBody.h>

#include <simulation/constraints/FixedRotationalJoint.h>


using namespace Eigen;


FixedRotationalJointDemo::FixedRotationalJointDemo(ApplicationControl& ac)
    : mAc(ac)
{

}

std::string FixedRotationalJointDemo::getName()
{
    return "Fixed Rotational Joint";
}

void FixedRotationalJointDemo::load()
{
    // upper cuboid
    SGLeafNode* node1 = mAc.getSGControl()->createBox(
                "Upper Cuboid", mAc.getSGControl()->getSceneGraph()->getRoot(),
                Vector(0.0,  0.0, 0.0), 2, 1, 1, true);
    mAc.getSGControl()->createRigidBody(node1->getData(), 1.0, false);
    RigidBody* rb1 = static_cast<RigidBody*>(node1->getData()->getSimulationObjectRaw());
    rb1->setRotationalDamping(0.005);
    rb1->setTranslationalDamping(0.005);

    // lower cuboid
    SGLeafNode* node2 = mAc.getSGControl()->createBox(
                "Lower Cuboid", mAc.getSGControl()->getSceneGraph()->getRoot(),
                Vector(0.0, -2.0, 0.0), 1, 1.5, 2, true);
    mAc.getSGControl()->createRigidBody(node2->getData(), 1.0, false);
    RigidBody* rb2 = static_cast<RigidBody*>(node2->getData()->getSimulationObjectRaw());
    rb2->setRotationalDamping(0.005);
    rb2->setTranslationalDamping(0.005);

    // plane joint connecting upper and lower cuboid
    std::shared_ptr<FixedRotationalJoint> planeJoint =
            std::make_shared<FixedRotationalJoint>(rb1, rb2);

    mAc.getSimulationControl()->addConstraint(planeJoint);

    mAc.getSimulationControl()->setGravity(Vector::Zero());
}

void FixedRotationalJointDemo::unload()
{

}
