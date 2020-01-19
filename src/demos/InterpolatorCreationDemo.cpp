#include "InterpolatorCreationDemo.h"

#include <ApplicationControl.h>

#include <simulation/rigid/RigidBody.h>

#include <simulation/constraints/BallJoint.h>

#include <simulation/forces/LinearForce.h>

#include <scene/data/geometric/GeometricDataFactory.h>
#include <scene/data/geometric/GeometricPoint.h>
#include <scene/data/geometric/Polygon2D.h>

#include <scene/model/RenderModel.h>

using namespace Eigen;

InterpolatorCreationDemo::InterpolatorCreationDemo(ApplicationControl* ac)
    : mAc(ac)
{

}

std::string InterpolatorCreationDemo::getName()
{
    return "Interpolator Creation";
}

void InterpolatorCreationDemo::load()
{
    mAc->getSimulationControl()->setGravity(Vector::Zero());

    // Sphere (high resolution target)
    {
        double sphereDim = 0.4;

        SGLeafNode* targetNode = mAc->getSGControl()->createLeafNode(
                    "Sphere (detailed)",
                    mAc->getSGControl()->getSceneGraph()->getRoot(),
                    std::make_shared<Polygon2D>(
                        GeometricDataFactory::create2DSphere(sphereDim, 4)),
                    Vector(0.0, 0.0, 0.0), true);

        targetNode->getData()->getRenderModel()->setWireframeEnabled(true);
    }

    // Box (low resolution source)
    {
        double boxDim = 0.8;

        // Create 3d box.
        SGLeafNode* sourceNode = mAc->getSGControl()->createLeafNode(
                    "Box", mAc->getSGControl()->getSceneGraph()->getRoot(),
                    std::make_shared<Polygon3D>(
                        GeometricDataFactory::create3DBox(boxDim, boxDim, boxDim)),
                    Vector(0.0, 0.0, 0.0), true);

        // Scale down a bit and transform to mid.
        std::shared_ptr<Polygon> poly1 =
                std::static_pointer_cast<Polygon>(sourceNode->getData()->getGeometricData());

        Eigen::Vector mid = Eigen::Vector::Zero();
        for (size_t i = 0; i < poly1->getPositions().size(); ++i)
            mid += poly1->getPositions()[i];
        mid /= poly1->getPositions().size();

        Eigen::Affine3d scaling =
                Eigen::Translation3d(Eigen::Vector3d(0.0, 0.0, 0.0)) *
                Eigen::Scaling(1.0) *
                Eigen::Translation3d(-mid);
        poly1->transform(scaling);

        sourceNode->getData()->getRenderModel()->setWireframeEnabled(true);

        // Add as FEMObject to simulation.
        mAc->getSGControl()->createFEMObject(sourceNode->getData());
    }
}

void InterpolatorCreationDemo::unload()
{
}
