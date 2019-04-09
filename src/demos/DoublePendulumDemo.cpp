#include "DoublePendulumDemo.h"

#include <ApplicationControl.h>

#include <simulation/rigid/RigidBody.h>

#include <simulation/constraints/BallJoint.h>

#include <simulation/forces/LinearForce.h>

#include <scene/data/geometric/GeometricPoint.h>

using namespace Eigen;

DoublePendulumDemo::DoublePendulumDemo(ApplicationControl& ac)
    : mAc(ac)
{

}

std::string DoublePendulumDemo::getName()
{
    return "Double Pendulum Demo";
}

void DoublePendulumDemo::load()
{
    mAc.getSimulationControl()->setStepSize(0.001);

    // upper cuboid
    SGLeafNode* node1 = mAc.getSGControl()->createBox(
                "Upper Cuboid", mAc.getSGControl()->getSceneGraph()->getRoot(),
                Vector(0.5,  0.0, 0.0), 1.0, 0.3, 0.3, true);
    mAc.getSGControl()->createRigidBody(node1->getData(), 1.0, false);
    RigidBody* rb1 = static_cast<RigidBody*>(node1->getData()->getSimulationObjectRaw());
    rb1->setRotationalDamping(0.001);
    rb1->setTranslationalDamping(0.001);

    // lower cuboid
    SGLeafNode* node2 = mAc.getSGControl()->createBox(
                "Lower Cuboid", mAc.getSGControl()->getSceneGraph()->getRoot(),
                Vector(-0.5, 0.0, 0.0), 1.0, 0.3, 0.3, true);
    mAc.getSGControl()->createRigidBody(node2->getData(), 1.0, false);
    RigidBody* rb2 = static_cast<RigidBody*>(node2->getData()->getSimulationObjectRaw());
    rb2->setRotationalDamping(0.001);
    rb2->setTranslationalDamping(0.001);

    // ball joint connecting upper and lower cuboid
    std::shared_ptr<BallJoint> ballJoint = std::make_shared<BallJoint>(
                SimulationPointRef(rb1, rb1->getPolygon().get(), Eigen::Vector3d(-0.5 ,0.0, 0.0)),
                SimulationPointRef(rb2, rb2->getPolygon().get(), Eigen::Vector3d(0.5 ,0.0, 0.0)));
    mAc.getSimulationControl()->addConstraint(ballJoint);

    // add linear force
    SGControl* sgControl = mAc.getSGControl();
    sgControl->createLinearForce("Linear Force",
                                 sgControl->getSceneGraph()->getRoot(),
                                 SimulationPointRef(rb1, rb1->getPolygon().get(),
                                                    Eigen::Vector3d(0.5 ,0.0, 0.0)),
                                 Vector(1.5, 0.0, 0.0),
                                 50.0);
}

void DoublePendulumDemo::unload()
{
}
