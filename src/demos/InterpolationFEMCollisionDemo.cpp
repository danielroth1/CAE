#include "InterpolationFEMCollisionDemo.h"

#include "ApplicationControl.h"
#include <data_structures/DataStructures.h>
#include <io/ImageLoader.h>
#include <io/importers/OBJImporter.h>
#include <modules/mesh_converter/MeshCriteria.h>
#include <rendering/Appearance.h>
#include <rendering/Appearances.h>
#include <rendering/Renderer.h>
#include <rendering/Texture.h>
#include <rendering/TextureUtils.h>
#include <scene/data/geometric/GeometricDataFactory.h>
#include <scene/data/geometric/GeometricDataListener.h>
#include <scene/data/geometric/MeshInterpolationManager.h>
#include <scene/data/geometric/MeshInterpolatorFEM.h>
#include <scene/data/geometric/Polygon2D.h>
#include <scene/data/geometric/Polygon3D.h>
#include <scene/model/MeshInterpolatorRenderModel.h>
#include <scene/model/PolygonRenderModel.h>
#include <ui/UIControl.h>

#include <math.h>

#include <simulation/fem/FEMObject.h>

#include <modules/interpolator/InterpolatorModule.h>

using namespace Eigen;

InterpolationFEMCollisionDemo::InterpolationFEMCollisionDemo(ApplicationControl& ac)
    : mAc(ac)
{

}

std::string InterpolationFEMCollisionDemo::getName()
{
    return "Interpolation FEM (Falling Spheres)";
}

void InterpolationFEMCollisionDemo::load()
{
    SGControl* sg = mAc.getSGControl();

    MeshCriteria criteria(0.0, 0.0, 0.0, 0.0, 0.0, true, 0.0);
    // Deforming sphere 1
    {
        double boxDim = 0.8;

        Eigen::Vector3d spherePos = Eigen::Vector(-1.0, 9.0, 0.0);

        // add target sphere
        double sphereDim = 0.4;
        SGLeafNode* targetNode = sg->createLeafNode(
                    "Sphere (detailed)",
                    sg->getSceneGraph()->getRoot(),
                    std::make_shared<Polygon2D>(
                        GeometricDataFactory::create2DSphere(sphereDim, 4)),
                    Eigen::Vector3d::Zero(), true);

        targetNode->getData()->getRenderModel()->setWireframeEnabled(true);
        targetNode->getData()->getGeometricData()->transform(Eigen::Affine3d(Eigen::Translation3d(spherePos)));

        // load the source box
        // use a 3d box
        SGLeafNode* sourceNode = sg->createLeafNode(
                    "Box", sg->getSceneGraph()->getRoot(),
                    std::make_shared<Polygon3D>(
//                        GeometricDataFactory::create3DBox(boxDim, boxDim, boxDim)),
                        GeometricDataFactory::create3DBox(boxDim, boxDim, boxDim, criteria)),
                    Eigen::Vector3d::Zero(), true);
//        sourceNode->getData()->getGeometricData()->transform(Eigen::Affine3d(Eigen::AngleAxisd(45.0 * M_PI / 180.0, Eigen::Vector3d(0.0, 0.0, 1.0))));
        sourceNode->getData()->getGeometricData()->transform(Eigen::Affine3d(Eigen::Translation3d(spherePos)));

        // scale down a bit
        std::shared_ptr<AbstractPolygon> poly1 =
                std::static_pointer_cast<AbstractPolygon>(sourceNode->getData()->getGeometricData());

        Eigen::Vector mid = Eigen::Vector::Zero();
        for (size_t i = 0; i < poly1->getPositions().size(); ++i)
            mid += poly1->getPositions()[i];
        mid /= poly1->getPositions().size();

//        Eigen::Affine3d scaling =
//                Eigen::Translation3d(mid) *
//                Eigen::Scaling(1.0) *
//                Eigen::Translation3d(-mid);
//        poly1->transform(scaling);
//        poly1->update();

        sourceNode->getData()->getRenderModel()->setWireframeEnabled(true);

        // interpolate
        std::shared_ptr<MeshInterpolatorFEM> interpolator =
                addInterpolation(sourceNode, targetNode);

        // Simulation:

        // add source as FEM object
        sg->createFEMObject(sourceNode->getData());
        ElasticMaterial material;
        material.setFromYoungsPoisson(100, 0.45);
        material.setPlasticYield(0.5);
        material.setPlasticCreep(10);
        material.setPlasticMaxStrain(10);
        static_cast<FEMObject*>(sourceNode->getData()->getSimulationObjectRaw())->setElasticMaterial(material);
        sg->createCollidable(sourceNode->getData(), interpolator);
//        static_cast<FEMObject*>(sourceNode->getData()->getSimulationObjectRaw())->setFrictionDynamic(0.0);

    }

    // Deforming sphere 2
    for (int i = 0; i < 2; ++i)
    {
        double boxDim = 0.8;

        Eigen::Vector3d spherePos = Eigen::Vector(-1.0, 13.0 + i * 0.85, 0.0);

        // add target sphere
        double sphereDim = 0.4;
        SGLeafNode* targetNode = sg->createLeafNode(
                    "Sphere (detailed)",
                    sg->getSceneGraph()->getRoot(),
                    std::make_shared<Polygon2D>(
                        GeometricDataFactory::create2DSphere(sphereDim, 4)),
                    spherePos, true);

        targetNode->getData()->getRenderModel()->setWireframeEnabled(true);

        // load the source box
        // use a 3d box
        SGLeafNode* sourceNode = sg->createLeafNode(
                    "Box", sg->getSceneGraph()->getRoot(),
                    std::make_shared<Polygon3D>(
//                        GeometricDataFactory::create3DBox(boxDim, boxDim, boxDim)),
                        GeometricDataFactory::create3DBox(boxDim, boxDim, boxDim, criteria)),
                    spherePos, true);

        // scale down a bit
        std::shared_ptr<AbstractPolygon> poly1 =
                std::static_pointer_cast<AbstractPolygon>(sourceNode->getData()->getGeometricData());

        Eigen::Vector mid = Eigen::Vector::Zero();
        for (size_t i = 0; i < poly1->getPositions().size(); ++i)
            mid += poly1->getPositions()[i];
        mid /= poly1->getPositions().size();

//        Eigen::Affine3d scaling =
//                Eigen::Translation3d(mid) *
//                Eigen::Scaling(1.0) *
//                Eigen::Translation3d(-mid);
//        poly1->transform(scaling);
//        poly1->update();

        sourceNode->getData()->getRenderModel()->setWireframeEnabled(true);

        // interpolate
        std::shared_ptr<MeshInterpolatorFEM> interpolator =
                addInterpolation(sourceNode, targetNode);

        // Simulation:

        // add source as FEM object
        sg->createFEMObject(sourceNode->getData());
        ElasticMaterial material;
        material.setFromYoungsPoisson(100, 0.45);
        material.setPlasticYield(0.5);
        material.setPlasticCreep(10);
        material.setPlasticMaxStrain(10);
        static_cast<FEMObject*>(sourceNode->getData()->getSimulationObjectRaw())->setElasticMaterial(material);
        sg->createCollidable(sourceNode->getData(), interpolator);

    }

    // Floor
    {
        SGLeafNode* floor = sg->createBox(
                    "Floor",
                    sg->getSceneGraph()->getRoot(),
                    Vector(0.0, -0.25, 0.0), 50, 0.5, 50, true);
        floor->getData()->getRenderModel()->setAppearances(
                    std::make_shared<Appearances>(
                        Appearance::createAppearanceFromColor(
        {0.8f, 0.8f, 0.8f, 1.0f})));

        sg->createRigidBody(floor->getData(), 1.0, true);
        sg->createCollidable(floor->getData());
    }

    // Wall
    {
        SGLeafNode* level = sg->createBox(
                    "Wall",
                    sg->getSceneGraph()->getRoot(),
                    Vector(0.0, 0.0, 0.0), 0.5, 3.0, 4.0, true);
        level->getData()->getGeometricData()->transform(
                    Eigen::Affine3d(
                        Eigen::Translation3d(Eigen::Vector3d(-3.0, 1.5, 0.0))));
        level->getData()->getGeometricData()->update();
        sg->createRigidBody(level->getData(), 1.0, true);
        sg->createCollidable(level->getData());
    }

    // Levels
    double plateWidth = 3.0;
    {
        SGLeafNode* level = sg->createBox(
                    "Level 1",
                    sg->getSceneGraph()->getRoot(),
                    Vector(0.0, 0.0, 0.0), 3, 0.1, plateWidth, true);
        level->getData()->getGeometricData()->transform(
                    Eigen::Translation3d(Eigen::Vector3d(1.0, 1.0, 0.0)) *
                    Eigen::AngleAxisd(0.125 * M_PI, Eigen::Vector3d(0.0, 0.0, 1.0)));
        level->getData()->getGeometricData()->update();
        sg->createRigidBody(level->getData(), 1.0, true);
        sg->createCollidable(level->getData());
    }

    {
        SGLeafNode* level = sg->createBox(
                    "Level 2",
                    sg->getSceneGraph()->getRoot(),
                    Vector(0.0, 0.0, 0.0), 3, 0.1, plateWidth, true);
        level->getData()->getGeometricData()->transform(
                    Eigen::Translation3d(Eigen::Vector3d(-1.0, 3.0, 0.0)) *
                    Eigen::AngleAxisd(-0.125 * M_PI, Eigen::Vector3d(0.0, 0.0, 1.0)));
        level->getData()->getGeometricData()->update();
        sg->createRigidBody(level->getData(), 1.0, true);
        sg->createCollidable(level->getData());
    }

    {
        SGLeafNode* level = sg->createBox(
                    "Level 3",
                    sg->getSceneGraph()->getRoot(),
                    Vector(0.0, 0.0, 0.0), 3, 0.1, plateWidth, true);
        level->getData()->getGeometricData()->transform(
                    Eigen::Translation3d(Eigen::Vector3d(1.0, 5.0, 0.0)) *
                    Eigen::AngleAxisd(0.125 * M_PI, Eigen::Vector3d(0.0, 0.0, 1.0)));
        level->getData()->getGeometricData()->update();
        sg->createRigidBody(level->getData(), 1.0, true);
        sg->createCollidable(level->getData());
    }

    {
        SGLeafNode* level = sg->createBox(
                    "Level 4",
                    sg->getSceneGraph()->getRoot(),
                    Vector(0.0, 0.0, 0.0), 3, 0.1, plateWidth, true);
        level->getData()->getGeometricData()->transform(
                    Eigen::Translation3d(Eigen::Vector3d(-1.0, 7.0, 0.0)) *
                    Eigen::AngleAxisd(-0.125 * M_PI, Eigen::Vector3d(0.0, 0.0, 1.0)));
        level->getData()->getGeometricData()->update();
        sg->createRigidBody(level->getData(), 1.0, true);
        sg->createCollidable(level->getData());
    }
}

std::shared_ptr<MeshInterpolatorFEM> InterpolationFEMCollisionDemo::addInterpolation(
            SGLeafNode* sourceNode,
            SGLeafNode* targetNode)
{
    return std::static_pointer_cast<MeshInterpolatorFEM>(
                mAc.getInterpolatorModule()->addInterpolator(
                    sourceNode, targetNode, MeshInterpolator::Type::FEM));
}

void InterpolationFEMCollisionDemo::unload()
{

}
