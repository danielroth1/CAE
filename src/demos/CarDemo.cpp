#include "CarDemo.h"

#include <simulation/rigid/RigidBody.h>

#include <ApplicationControl.h>

#include <scene/data/geometric/GeometricDataFactory.h>
#include <scene/data/geometric/Polygon2D.h>

#include <simulation/forces/LinearForce.h>

#include <simulation/constraints/BallJoint.h>
#include <simulation/constraints/DistanceJoint.h>
#include <simulation/constraints/FixedRotationalJoint.h>
#include <simulation/constraints/LineJoint.h>

using namespace Eigen;

CarDemo::CarDemo(ApplicationControl& ac)
    : mAc(ac)
{
    mSg = mAc.getSGControl();
}

std::string CarDemo::getName()
{
    return "Car";
}

void CarDemo::load()
{
//    mAc.getSimulationControl()->setGravity(Vector::Zero());

    // Floor
    SGLeafNode* floor = mAc.getSGControl()->createBox(
                "Floor",
                mAc.getSGControl()->getSceneGraph()->getRoot(),
                Vector(0.0, -3.0, 0.0), 12, 0.5, 12, true);
    mAc.getSGControl()->createRigidBody(floor->getData(), 1.0, true);
    mAc.getSGControl()->createCollidable(floor->getData());

    createCar(Affine3d::Identity(), 2.0, 4.0, 1.0, 0.2, 0.5, 100.0, 0.0);
}

void CarDemo::unload()
{

}

void CarDemo::createCar(
        Eigen::Affine3d transformation,
        double width,  double length, double height,
        double tireWidth, double springLength,
        double cSpring, double cDamping)
{

    SGChildrenNode* carNode = mSg->createChildrenNode(
                mSg->getSceneGraph()->getRoot(), "Car");

    // create hull
    SGLeafNode* hull = mSg->createBox(
                "Hull", carNode, transformation * Vector::Zero(),
                width, height, length);
    std::shared_ptr<RigidBody> hullRigid = mSg->createRigidBody(
                hull->getData(), 1.0, false);

    // create 4 tires
    for (int i = 0; i < 4; ++i)
    {
        Vector position;
        std::string name;
        switch(i)
        {
        case 0:
            position = Vector(-0.5 * width, -0.5 * height, 0.5 * length);
            name = "Left front tire";
            break;
        case 1:
            position = Vector(0.5 * width, -0.5 * height, 0.5 * length);
            name = "Right front tire";
            break;
        case 2:
            position = Vector(-0.5 * width, -0.5 * height, -0.5 * length);
            name = "Left back tire";
            break;
        case 3:
            position = Vector(0.5 * width, -0.5 * height, -0.5 * length);
            name = "Right back tire";
            break;
        }

        createTire(carNode,
                   name,
                   transformation,
                   SimulationPointRef(hullRigid.get(), hullRigid->getPolygon().get(),
                                      position),
                   tireWidth, springLength, cSpring, cDamping);

    }

}

void CarDemo::createTire(
        SGChildrenNode* parent,
        std::string name,
        Affine3d transformation,
        const SimulationPointRef& target,
        double tireWidth,
        double springLength,
        double cSpring,
        double cDamping)
{
    Vector position = target.getGeometricPoint() - Vector(0.0, springLength, 0.0);

    std::shared_ptr<Polygon2D> tireTemplate =
            std::make_shared<Polygon2D>(
                GeometricDataFactory::create2DSphere(2 * tireWidth, 3));

    // left front tire
    std::shared_ptr<Polygon2D> tirePolyLf = std::make_shared<Polygon2D>(*tireTemplate.get());
    tirePolyLf->transform(transformation * Translation3d(position));
    SGLeafNode* tireLf = mSg->createLeafNode(
                name,
                parent,
                tirePolyLf,
                true);
    std::shared_ptr<RigidBody> rigidLf =
            mSg->createRigidBody(tireLf->getData(), 0.1, false);
    mSg->createCollidable(tireLf->getData());

    // spring
    SimulationPointRef source = SimulationPointRef(
                rigidLf.get(), rigidLf->getPolygon().get(), Vector::Zero());
    std::shared_ptr<LinearForce> f = std::make_shared<LinearForce>(
                source,
                target, cSpring, cDamping, springLength);
    mAc.getSimulationControl()->addLinearForce(f);

    // fixed rotational
//    std::shared_ptr<FixedRotationalJoint> fixedJoint =
//            std::make_shared<FixedRotationalJoint>(
//                rigidLf,
//                std::static_pointer_cast<RigidBody>(target.getSimulationObject()));
//    mAc.getSimulationControl()->addConstraint(fixedJoint);

    // line joint
    std::shared_ptr<LineJoint> lineJoint =
            std::make_shared<LineJoint>(target, source, Vector(0, -1.0, 0));
    mAc.getSimulationControl()->addConstraint(lineJoint);

//    std::shared_ptr<BallJoint> ballJoint =
//            std::make_shared<BallJoint>(target, source, springLength);
//    mAc.getSimulationControl()->addConstraint(ballJoint);

//    std::shared_ptr<DistanceJoint> distanceJoint =
//            std::make_shared<DistanceJoint>(target, source, springLength);
//    mAc.getSimulationControl()->addConstraint(distanceJoint);

}
