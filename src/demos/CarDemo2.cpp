#include "CarDemo2.h"


#include <ApplicationControl.h>

#include <scene/data/geometric/GeometricDataFactory.h>
#include <scene/data/geometric/MeshInterpolatorFEM.h>
#include <scene/data/geometric/Polygon2D.h>
#include <scene/model/PolygonRenderModel.h>
#include <scene/model/RenderModel.h>

#include <modules/interpolator/InterpolatorModule.h>
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

CarDemo2::CarDemo2(ApplicationControl& ac)
    : mAc(ac)
{
    mSg = mAc.getSGControl();
}

std::string CarDemo2::getName()
{
    return "Car (complex)";
}

void CarDemo2::load()
{
    mAc.getSimulationControl()->setGravity(Vector::Zero());

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
        SGChildrenNode* root = mAc.getSGControl()->getSceneGraph()->getRoot();
//        std::string path = "/home/daniel/models/AC Cobra/Shelby.obj";
//        SGLeafNode* carNode = static_cast<SGLeafNode*>(
//                    mAc.getSGControl()->importFileAsChild(File(path), root, false));
//        carNode->getData()->setVerticesSelectable(false);

//        std::shared_ptr<GeometricData> gd = carNode->getData()->getGeometricData();
//        gd->updateBoundingBox();
//        BoundingBox bb2 = gd->getBoundingBox();
        BoundingBox bb2;
        bb2.min() = Eigen::Vector(-1.76589, 0, -1.39474);
        bb2.max() = Eigen::Vector(4.76877, 1.92912, 1.37845);
        bb2.size() = Eigen::Vector(6.53466, 1.92912, 2.77319);
        SGLeafNode* boundingBox = mAc.getSGControl()->createBox(
                    "Shelby (BoundingBox)",
                    root,
                    bb2.mid(), bb2.size()[0], bb2.size()[1], bb2.size()[2]);

        MeshCriteria criteria(0, 0, 0, 0, 0, true, 60.0);
        SGLeafNode* boundingBox3D =
                mAc.getSGControl()->create3DGeometryFrom2D(boundingBox, criteria);
        boundingBox3D->getData()->getRenderModel()->setWireframeEnabled(true);

//        std::shared_ptr<MeshInterpolatorFEM> interpolator =
//                std::static_pointer_cast<MeshInterpolatorFEM>(
//                    mAc.getInterpolatorModule()->addInterpolator(
//                    boundingBox3D, carNode, MeshInterpolator::Type::FEM));

//        std::shared_ptr<PolygonRenderModel> rm =
//                std::static_pointer_cast<PolygonRenderModel>(
//                    carNode->getData()->getRenderModel());
//        rm->setMeshInterpolator(interpolator);

        mAc.getSGControl()->createFEMObject(boundingBox3D->getData(), 1);
        std::shared_ptr<FEMObject> femObj =
                std::dynamic_pointer_cast<FEMObject>(
                    boundingBox3D->getData()->getSimulationObject());
        femObj->setYoungsModulus(5e+2);
        mAc.getSGControl()->createCollidable(boundingBox3D->getData());

    }
}

void CarDemo2::unload()
{

}
