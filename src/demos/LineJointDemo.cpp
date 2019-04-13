#include "LineJointDemo.h"

#include <ApplicationControl.h>

#include <simulation/rigid/RigidBody.h>

#include <simulation/constraints/BallJoint.h>
#include <simulation/constraints/LineJoint.h>
#include <simulation/constraints/PlaneJoint.h>

#include <simulation/forces/LinearForce.h>

#include <scene/data/geometric/GeometricPoint.h>

using namespace Eigen;

LineJointDemo::LineJointDemo(ApplicationControl& ac)
    : mAc(ac)
{

}

std::string LineJointDemo::getName()
{
    return "Line Joint";
}

void LineJointDemo::load()
{
    mAc.getSimulationControl()->setStepSize(0.001);
    mAc.getSimulationControl()->setGravity(Eigen::Vector::Zero());

    double yOffset = 2.3;

    double width = 0.3;
    double length = 5.0;

    // upper cuboid
    SGLeafNode* node1 = mAc.getSGControl()->createBox(
                "Upper Cuboid", mAc.getSGControl()->getSceneGraph()->getRoot(),
                Vector(0.0,  0.0 + yOffset, 0.0), length, width, width, true);
    mAc.getSGControl()->createRigidBody(node1->getData(), 1.0, false);
    RigidBody* rb1 = static_cast<RigidBody*>(node1->getData()->getSimulationObjectRaw());
    rb1->setRotationalDamping(0.005);
    rb1->setTranslationalDamping(0.005);

    // lower cuboid
    double length2 = 0.6;

    SGLeafNode* node2 = mAc.getSGControl()->createBox(
                "Lower Cuboid", mAc.getSGControl()->getSceneGraph()->getRoot(),
                Vector(0.0, -width/2.0 - length2/2.0 + yOffset, 0.0), width, length2, width, true);
    mAc.getSGControl()->createRigidBody(node2->getData(), 1.0, false);
    RigidBody* rb2 = static_cast<RigidBody*>(node2->getData()->getSimulationObjectRaw());
    rb2->setRotationalDamping(0.005);
    rb2->setTranslationalDamping(0.005);

    // line joint connecting upper and lower cuboid
    std::shared_ptr<LineJoint> lineJoint = std::make_shared<LineJoint>(
                SimulationPointRef(rb1,
                                   rb1->getPolygon().get(),
                                   Eigen::Vector3d(0.0, -width / 2.0, 0.0)),
                SimulationPointRef(rb2,
                                   rb2->getPolygon().get(),
                                   Eigen::Vector3d(0.0, length2 / 2.0, 0.0)),
                Eigen::Vector(1.0, 0.0, 0.0));

    mAc.getSimulationControl()->addConstraint(lineJoint);

    // fixate first part in origin
    SGLeafNode* point = mAc.getSGControl()->createSimulationPoint(
                "Fixed point",
                mAc.getSGControl()->getSceneGraph()->getRoot(),
                rb1->getPosition());
    mAc.getSimulationControl()->addConstraint(
                std::make_shared<BallJoint>(
                    SimulationPointRef(
                        rb1, rb1->getPolygon().get(), Eigen::Vector::Zero()),
                    SimulationPointRef(
                        point->getData()->getSimulationObjectRaw(), 0)));

    // add linear force to second part
    SGControl* sgControl = mAc.getSGControl();
    sgControl->createLinearForce(
                "Linear Force",
                sgControl->getSceneGraph()->getRoot(),
                SimulationPointRef(rb2, rb2->getPolygon().get(),
                                   Eigen::Vector3d(0.0, -length2 / 2.0, 0.0)),
                Vector(0.0, -length2-width-0.3 + yOffset, 0.0),
                10.0);
}

void LineJointDemo::unload()
{
}
