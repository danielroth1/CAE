#include "CarDemo.h"


#include <ApplicationControl.h>

#include <scene/data/geometric/GeometricDataFactory.h>
#include <scene/data/geometric/Polygon2D.h>
#include <scene/model/RenderModel.h>

#include <modules/mesh_converter/MeshCriteria.h>
#include <simulation/forces/LinearForce.h>
#include <simulation/forces/RotationalMotor.h>
#include <simulation/rigid/RigidBody.h>

#include <simulation/constraints/BallJoint.h>
#include <simulation/constraints/DistanceJoint.h>
#include <simulation/constraints/DoubleAxisRotationalJoint.h>
#include <simulation/constraints/FixedRotationalJoint.h>
#include <simulation/constraints/LineJoint.h>

#include <rendering/Appearance.h>
#include <rendering/Appearances.h>


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
    {
        SGLeafNode* floor = mAc.getSGControl()->createBox(
                    "Floor",
                    mAc.getSGControl()->getSceneGraph()->getRoot(),
                    Vector(-15.0, -4.5, 0.0), 40, 0.5, 20, true);

        floor->getData()->getRenderModel()->setAppearances(
                    std::make_shared<Appearances>(
                        Appearance::createAppearanceFromColor(
        {0.8f, 0.8f, 0.8f, 1.0f})));

        mAc.getSGControl()->createRigidBody(floor->getData(), 1.0, true);
        mAc.getSGControl()->createCollidable(floor->getData());
    }

    // Car
    {
        SGChildrenNode* node = createCar(
                    Affine3d::Identity(), 2.0, 4.0, 1.0, 0.2, 0.5, 1000.0, 100.0);

        // Rotate by 180 degree.
        Eigen::Affine3d transformation =
                Affine3d::Identity() * Eigen::AngleAxisd(1.5 * 3.14, Eigen::Vector3d(0.0, 1.0, 0.0));
        class TransformVisitor : public SGNodeVisitorImpl
        {
        public:
            TransformVisitor(const Eigen::Affine3d& _transformation)
                : transformation(_transformation)
            {

            }

            virtual void visit(SGChildrenNode* /*childrenNode*/)
            {

            }

            virtual void visit(SGLeafNode* leafNode)
            {
                leafNode->getData()->getSimulationObject()->transform(transformation);
                leafNode->getData()->getSimulationObject()->updateGeometricData();
            }

            Eigen::Affine3d transformation;
        } visitor(transformation);

        SGTraverser traverser(node);
        traverser.traverse(visitor);
    }

    // Boxes
    {
        Eigen::Affine3d transformation =
                Eigen::Translation3d(-15.0, -3.0, 0.0) *
                Eigen::AngleAxisd(0.5 * 3.14, Eigen::Vector3d(0.0, 1.0, 0.0));

        double cubeMass = 0.2;
        for (int r = 0; r < 8; ++r)
        {
            for (int c = 0; c < 9; ++c)
            {
                for (int z = 0; z < 1; ++z)
                {
                    if (false && r % 2 == 1)
                    {
                        MeshCriteria criteria(0.0, 0.0, 0.0, 0.0, 0.0, true, 0.0);

                        std::shared_ptr<Polygon3D> poly3 =
                                std::make_shared<Polygon3D>(
                                    GeometricDataFactory::create3DBox(0.5, 0.5, 0.5));
//                        SGLeafNode* node1 = mAc.getSGControl()->createLeafNode(
//                                    "Box", mAc.getSGControl()->getSceneGraph()->getRoot(),
//                                    poly3, Vector(-2 + 0.6 * c, -0.5 + 0.6 * r, -1 + 0.6 * z), true);

                        SGLeafNode* node1 = mAc.getSGControl()->createBox(
                                    "Box", mAc.getSGControl()->getSceneGraph()->getRoot(),
                                    Vector(-2 + 0.6 * c, -0.5 + 0.6 * r, -1 + 0.6 * z),
                                    0.5, 0.5, 0.5, true);
                        mAc.getSGControl()->create3DGeometryFrom2D(node1, criteria);
                        mAc.getSGControl()->createFEMObject(node1->getData(), cubeMass);

                        node1->getData()->getSimulationObject()->transform(transformation);
                        node1->getData()->getSimulationObject()->updateGeometricData();

                        mAc.getSGControl()->createCollidable(node1->getData());

                        std::shared_ptr<FEMObject> femObj =
                                std::dynamic_pointer_cast<FEMObject>(
                                    node1->getData()->getSimulationObject());
                        femObj->setYoungsModulus(30000);

                    }
                    else
                    {
                        SGLeafNode* node1 = mAc.getSGControl()->createBox(
                                    "Box", mAc.getSGControl()->getSceneGraph()->getRoot(),
                                    Vector(-2 + 0.6 * c, -0.5 + 0.6 * r, -1 + 0.6 * z),
                                    0.5, 0.5, 0.5, true);
                        mAc.getSGControl()->createRigidBody(node1->getData(), cubeMass, false);

                        node1->getData()->getSimulationObject()->transform(transformation);
                        node1->getData()->getSimulationObject()->updateGeometricData();

                        mAc.getSGControl()->createCollidable(node1->getData());

//                        if (c == 1)
//                        {
//                            mAc.getSGControl()->createLinearForce(
//                                        "Linear Force",
//                                        mAc.getSGControl()->getSceneGraph()->getRoot(),
//                                        SimulationPointRef(node1->getData()->getSimulationObject().get(), 0),
//                                        node1->getData()->getSimulationObject()->getPosition(0) + Eigen::Vector3d(1, -3, 0), 10.0);
//                        }
                    }
                }

            }
        }
    }

}

void CarDemo::unload()
{

}

SGChildrenNode* CarDemo::createCar(
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

    hull->getData()->getRenderModel()->setAppearances(
                std::make_shared<Appearances>(
                    Appearance::createAppearanceFromColor(
    {1.0f, 0.1f, 0.1f, 1.0f})));

    std::shared_ptr<RigidBody> hullRigid = mSg->createRigidBody(
                hull->getData(), 5.0, false);
    mSg->createCollidable(hull->getData(), 0.05);

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
                   SimulationPointRef(hullRigid.get(),
                                      hullRigid->getPolygon().get(),
                                      position),
                   tireWidth, springLength, cSpring, cDamping);

    }

    return carNode;
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

    // hull
    std::shared_ptr<RigidBody> hull =
            std::static_pointer_cast<RigidBody>(target.getSimulationObject());

    // left front tire
    std::shared_ptr<Polygon2D> tirePolyLf = std::make_shared<Polygon2D>(*tireTemplate.get());
    tirePolyLf->transform(transformation * Translation3d(position));
    SGLeafNode* tire = mSg->createLeafNode(
                name,
                parent,
                tirePolyLf,
                Eigen::Vector::Zero(),
                true);

    tire->getData()->getRenderModel()->setAppearances(
                std::make_shared<Appearances>(
                    Appearance::createAppearanceFromColor(
    {0.1f, 0.1f, 0.1f, 1.0f})));

    std::shared_ptr<RigidBody> rigid =
            mSg->createRigidBody(tire->getData(), 1.0, false);
    mSg->createCollidable(tire->getData(), 0.04);

    // spring
    SimulationPointRef source = SimulationPointRef(
                rigid.get(), rigid->getPolygon().get(), Vector::Zero());
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

    // Double Rotational Joint
    std::shared_ptr<DoubleAxisRotationalJoint> rotLineJoint =
            std::make_shared<DoubleAxisRotationalJoint>(
                rigid, hull, Vector(1.0, 0.0, 0.0), Vector(1.0, 0.0, 0.0));
    mAc.getSimulationControl()->addConstraint(rotLineJoint);

    // Rotational motor
    std::shared_ptr<RotationalMotor> motorForce =
            std::make_shared<RotationalMotor>(
                rigid, hull, Vector(1.0, 0.0, 0.0), 8.0);
    mAc.getSimulationControl()->addForce(motorForce);

//    std::shared_ptr<BallJoint> ballJoint =
//            std::make_shared<BallJoint>(target, source, springLength);
//    mAc.getSimulationControl()->addConstraint(ballJoint);

//    std::shared_ptr<DistanceJoint> distanceJoint =
//            std::make_shared<DistanceJoint>(target, source, springLength);
//    mAc.getSimulationControl()->addConstraint(distanceJoint);

}
