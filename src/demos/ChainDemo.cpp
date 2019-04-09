#include "ChainDemo.h"

#include <ApplicationControl.h>

#include <simulation/rigid/RigidBody.h>

#include <simulation/constraints/BallJoint.h>

#include <simulation/forces/LinearForce.h>

#include <scene/data/geometric/GeometricPoint.h>

using namespace Eigen;

ChainDemo::ChainDemo(ApplicationControl& ac)
    : mAc(ac)
{

}

std::string ChainDemo::getName()
{
    return "Chain Demo";
}

void ChainDemo::load()
{
    mAc.getSimulationControl()->setStepSize(0.001);

    int nChainParts = 25;
    double cpl = 0.4; // chain part length
    double cl = nChainParts * cpl; // chain length

    // upper cuboid
    SGLeafNode* node1 = mAc.getSGControl()->createBox(
                "Upper Cuboid", mAc.getSGControl()->getSceneGraph()->getRoot(),
                Vector(cl / 2.0,  0.0, 0.0), cpl, 0.3, 0.3, true);
    mAc.getSGControl()->createRigidBody(node1->getData(), 1.0, false);
    RigidBody* rb1 = static_cast<RigidBody*>(node1->getData()->getSimulationObjectRaw());
    rb1->setRotationalDamping(0.001);
    rb1->setTranslationalDamping(0.001);

    // lower cuboid
    RigidBody* prevChainPart = rb1;
    for (int i = 1; i < nChainParts; ++i)
    {
        double pos = cl / 2.0 - i * cpl;

        SGLeafNode* node2 = mAc.getSGControl()->createBox(
                    "Lower Cuboid", mAc.getSGControl()->getSceneGraph()->getRoot(),
                    Vector(pos, 0.0, 0.0), cpl, 0.3, 0.3, true);
        mAc.getSGControl()->createRigidBody(node2->getData(), 1.0, false);
        RigidBody* rb2 = static_cast<RigidBody*>(node2->getData()->getSimulationObjectRaw());
        rb2->setRotationalDamping(0.001);
        rb2->setTranslationalDamping(0.001);

        // ball joint connecting upper and lower cuboid
        std::shared_ptr<BallJoint> ballJoint = std::make_shared<BallJoint>(
                    SimulationPointRef(prevChainPart,
                                       prevChainPart->getPolygon().get(),
                                       Eigen::Vector3d(-cpl / 2.0, 0.0, 0.0)),
                    SimulationPointRef(rb2,
                                       rb2->getPolygon().get(),
                                       Eigen::Vector3d(cpl / 2.0, 0.0, 0.0)));
        mAc.getSimulationControl()->addConstraint(ballJoint);

        prevChainPart = rb2;
    }


    // add linear force
    SGControl* sgControl = mAc.getSGControl();
    sgControl->createLinearForce("Linear Force",
                                 sgControl->getSceneGraph()->getRoot(),
                                 SimulationPointRef(rb1, rb1->getPolygon().get(),
                                                    Eigen::Vector3d(cpl / 2.0, 0.0, 0.0)),
                                 Vector(cl / 2.0 + cpl, 0.0, 0.0),
                                 1000.0);
}

void ChainDemo::unload()
{
}
