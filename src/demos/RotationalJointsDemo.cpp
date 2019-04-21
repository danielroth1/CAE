#include "RotationalJointsDemo.h"

#include <ApplicationControl.h>

#include <simulation/rigid/RigidBody.h>

#include <simulation/constraints/AxisRotationalJoint.h>
#include <simulation/constraints/BallJoint.h>
#include <simulation/constraints/DoubleAxisRotationalJoint.h>
#include <simulation/constraints/FixedRotationalJoint.h>
#include <simulation/constraints/HingeJoint.h>


using namespace Eigen;


RotationalJointsDemo::RotationalJointsDemo(ApplicationControl& ac)
    : mAc(ac)
{

}

std::string RotationalJointsDemo::getName()
{
    return "Rotational Joints";
}

void RotationalJointsDemo::load()
{
    mAc.getSimulationControl()->setGravity(Vector::Zero());

    for (int i = 0; i < 4; ++i)
    {
        // upper cuboid
        SGLeafNode* node1 = mAc.getSGControl()->createBox(
                    "Upper Cuboid", mAc.getSGControl()->getSceneGraph()->getRoot(),
                    Vector(i*3, 0.0, 0.0), 1, 1, 1, true);
        mAc.getSGControl()->createRigidBody(node1->getData(), 1.0, false);
        std::shared_ptr<RigidBody> rb1 =
                std::static_pointer_cast<RigidBody>(
                    node1->getData()->getSimulationObject());
        rb1->setRotationalDamping(0.005);
        rb1->setTranslationalDamping(0.005);

        // fixate first part in origin
        SGLeafNode* point = mAc.getSGControl()->createSimulationPoint(
                    "Fixed point",
                    mAc.getSGControl()->getSceneGraph()->getRoot(),
                    rb1->getPosition());
        mAc.getSimulationControl()->addConstraint(
                    std::make_shared<BallJoint>(
                        SimulationPointRef(
                            rb1.get(), rb1->getPolygon().get(), Eigen::Vector::Zero()),
                        SimulationPointRef(
                            point->getData()->getSimulationObjectRaw(), 0)));

        // lower cuboid
        SGLeafNode* node2 = mAc.getSGControl()->createBox(
                    "Lower Cuboid", mAc.getSGControl()->getSceneGraph()->getRoot(),
                    Vector(i*3 + 1.0, 0.0, 0.0), 1, 1, 1, true);
        mAc.getSGControl()->createRigidBody(node2->getData(), 1.0, false);
        std::shared_ptr<RigidBody> rb2 =
                std::static_pointer_cast<RigidBody>(
                    node2->getData()->getSimulationObject());
        rb2->setRotationalDamping(0.005);
        rb2->setTranslationalDamping(0.005);

        // fixate second part in orign
        if (i < 3)
        {
            SGLeafNode* point2 = mAc.getSGControl()->createSimulationPoint(
                        "Fixed point",
                        mAc.getSGControl()->getSceneGraph()->getRoot(),
                        rb2->getPosition());
            mAc.getSimulationControl()->addConstraint(
                        std::make_shared<BallJoint>(
                            SimulationPointRef(
                                rb2.get(), rb2->getPolygon().get(), Eigen::Vector::Zero()),
                            SimulationPointRef(
                                point2->getData()->getSimulationObjectRaw(), 0)));
        }

        if (i == 0)
        {
            std::shared_ptr<FixedRotationalJoint> fixedRotationalJoint =
                    std::make_shared<FixedRotationalJoint>(rb1, rb2);
            mAc.getSimulationControl()->addConstraint(fixedRotationalJoint);
        }
        else if (i == 1)
        {
            std::shared_ptr<AxisRotationalJoint> axisRotatoinalJoint =
                    std::make_shared<AxisRotationalJoint>(
                        rb1, rb2, Eigen::Vector(0, 1, 0));
            mAc.getSimulationControl()->addConstraint(axisRotatoinalJoint);
        }
        else if (i == 2)
        {
            // plane joint connecting upper and lower cuboid
            std::shared_ptr<DoubleAxisRotationalJoint> doubleRotationalJoint =
                    std::make_shared<DoubleAxisRotationalJoint>(
                        rb1, rb2, Eigen::Vector(0, 0, 1), Eigen::Vector(0, 0, 1));
            mAc.getSimulationControl()->addConstraint(doubleRotationalJoint);

        }
        else if (i == 3)
        {
            // plane joint connecting upper and lower cuboid
            std::shared_ptr<HingeJoint> doubleRotationalJoint =
                    std::make_shared<HingeJoint>(
                        rb1, rb2,
                        Eigen::Vector(0.5, 0, 0), Eigen::Vector(-0.5, 0, 0),
                        Eigen::Vector(0, 0, 1), Eigen::Vector(0, 0, 1));
            mAc.getSimulationControl()->addConstraint(doubleRotationalJoint);
        }

    }
}

void RotationalJointsDemo::unload()
{

}
