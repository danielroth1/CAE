#include "BallJointDemo.h"

#include <ApplicationControl.h>

#include <simulation/rigid/RigidBody.h>

#include <simulation/constraints/BallJoint.h>

#include <simulation/forces/LinearForce.h>

#include <scene/data/geometric/GeometricPoint.h>

using namespace Eigen;

BallJointDemo::BallJointDemo(ApplicationControl& ac)
    : mAc(ac)
{

}

std::string BallJointDemo::getName()
{
    return "BallJoint Demo";
}

void BallJointDemo::load()
{
//    mAc.getSimulationControl()->setGravity(Eigen::Vector3d::Zero());

    // upper cuboid
    SGLeafNode* node1 = mAc.getSGControl()->createBox(
                "Upper Cuboid", mAc.getSGControl()->getSceneGraph()->getRoot(),
                Vector(0.5,  0.0, 0.0), 1.0, 0.3, 0.3, true);
    mAc.getSGControl()->createRigidBody(node1->getData(), 1.0, false);

    // lower cuboid
    SGLeafNode* node2 = mAc.getSGControl()->createBox(
                "Lower Cuboid", mAc.getSGControl()->getSceneGraph()->getRoot(),
                Vector(-0.5, 0.0, 0.0), 1.0, 0.3, 0.3, true);
    mAc.getSGControl()->createRigidBody(node2->getData(), 1.0, false);


    RigidBody* rb1 = static_cast<RigidBody*>(node1->getData()->getSimulationObjectRaw());
    RigidBody* rb2 = static_cast<RigidBody*>(node2->getData()->getSimulationObjectRaw());

    rb1->setRotationalDamping(0.0);
    rb1->setTranslationalDamping(0.0);
    rb2->setRotationalDamping(0.0);
    rb2->setTranslationalDamping(0.0);

    std::shared_ptr<BallJoint> ballJoint = std::make_shared<BallJoint>(
                SimulationPointRef(rb1, rb1->getPolygon().get(), Eigen::Vector3d(-0.5 ,0.0, 0.0)),
                SimulationPointRef(rb2, rb2->getPolygon().get(), Eigen::Vector3d(0.5 ,0.0, 0.0)));
    mAc.getSimulationControl()->addConstraint(ballJoint);

    // add linear force
    SGControl* sgControl = mAc.getSGControl();
    SGLeafNode* leafNode =
            sgControl->createAndAddLeafNodeToRoot("linearForcePoint");
    leafNode->setData(new SceneLeafData(leafNode));

    leafNode->getData()->setGeometricData(
                std::make_shared<GeometricPoint>(
                    Vector(1.0, 0.0, 0.0)));

    sgControl->createAndSetCorrespondingSimulationObject(
                leafNode);

    std::shared_ptr<LinearForce> lf = std::make_shared<LinearForce>(
                SimulationPointRef(rb1, rb1->getPolygon().get(), Eigen::Vector3d(0.5 ,0.0, 0.0)),
                SimulationPointRef(leafNode->getData()->getSimulationObjectRaw(), 0),
                50.0);
    mAc.getSimulationControl()->addLinearForce(lf);
}

void BallJointDemo::unload()
{
}
