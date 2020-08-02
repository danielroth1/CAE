#include "FallingObjectsDemo.h"

#include <scene/scene_graph/SGControl.h>
#include <scene/scene_graph/SGCore.h>

#include <modules/mesh_converter/MeshCriteria.h>

#include <ApplicationControl.h>
#include <QCoreApplication>
#include <SimulationControl.h>

#include <simulation/fem/FEMObject.h>

#include <io/importers/OBJImporter.h>

#include <scene/data/GeometricData.h>

#include <scene/model/RenderModel.h>

#include <rendering/Appearance.h>
#include <rendering/Appearances.h>

#include <scene/data/geometric/MeshInterpolationManager.h>
#include <scene/data/geometric/Polygon.h>
#include <scene/data/geometric/Polygon3D.h>

#include <modules/interpolator/InterpolatorModule.h>

using namespace Eigen;

FallingObjectsDemo::FallingObjectsDemo(ApplicationControl* ac,
                                       std::string name,
                                       bool rigid)
    : mAc(ac)
    , mName(name)
    , mRigid(rigid)
{

}

std::string FallingObjectsDemo::getName()
{
    return mName;
}

void FallingObjectsDemo::load()
{
    mAc->getSimulationControl()->setSimulationPaused(true);

    // Floor
    SGLeafNode* nodeFloor = mAc->getSGControl()->createBox(
                "Floor", mAc->getSGControl()->getSceneGraph()->getRoot(),
                Vector(0.0, -1.5, 0.0), 30, 0.5, 30, true);

    mAc->getSGControl()->createRigidBody(nodeFloor->getData(), 1.0, true);
    mAc->getSGControl()->createCollidable(nodeFloor->getData());

    double targetWidth = 0.42;

//    SGNode* node = mAc->getSGControl()->importFileAsChild(
//                File(QCoreApplication::applicationDirPath().toStdString() + "/assets/LibertStatue.obj"),
//                mAc->getSGControl()->getSceneGraph()->getRoot(),
//                false);

//    BoundingBox bb;
//    if (node->isLeaf())
//    {
//        SGLeafNode* leaf = static_cast<SGLeafNode*>(node);
//        std::shared_ptr<GeometricData> gd = leaf->getData()->getGeometricData();
//        gd->updateBoundingBox();
//        bb = gd->getBoundingBox();
//    }

//    importAndScale(mAc->getSGControl()->getSceneGraph()->getRoot(),
//                   "/home/daniel/objs/animals/Armadillo1k.off",
//                   {0.0, 0.0, 1.0, 1.0}, // blue
//                   targetWidth,
//                   Eigen::Affine3d(Eigen::Translation3d(0.0, 4.0, 0.0)),
//                   mRigid);

    importAndScale(mAc->getSGControl()->getSceneGraph()->getRoot(),
                   QCoreApplication::applicationDirPath().toStdString() + "/assets/animals/Armadillo40k.off",
                   {1.0, 0.0, 0.0, 1.0}, // red
                   targetWidth,
                   Eigen::Affine3d(Eigen::Translation3d(0.0, 1.0, 0.0)),
                   mRigid);

    importAndScale(mAc->getSGControl()->getSceneGraph()->getRoot(),
                   QCoreApplication::applicationDirPath().toStdString() + "/assets/animals/Bunny35k.off",
                   {1.0, 1.0, 0.0, 1.0}, // yellow
                   targetWidth,
                   Eigen::Affine3d(Eigen::Translation3d(0.0, 2.0, 0.0)),
                   mRigid);

    importAndScale(mAc->getSGControl()->getSceneGraph()->getRoot(),
                   QCoreApplication::applicationDirPath().toStdString() + "/assets/animals/Frog19k.off",
                   {0.0, 0.0, 1.0, 1.0}, // blue
                   targetWidth,
                   Eigen::Affine3d(Eigen::Translation3d(0.0, 4.0, 0.0)),
                   mRigid);

    mAc->getSimulationControl()->setSimulationPaused(false);
}

void FallingObjectsDemo::unload()
{

}

SGLeafNode* FallingObjectsDemo::importAndScale(
        SGChildrenNode* parent,
        std::string path,
        const std::array<float, 4>& color,
        double targetWidth,
        Eigen::Affine3d transform,
        bool rigid)
{
    SGNode* node = mAc->getSGControl()->importFileAsChild(File(path), parent, false);

    if (node->isLeaf())
    {
        // Target
        SGLeafNode* leaf = static_cast<SGLeafNode*>(node);
        leaf->getData()->setVerticesSelectable(false);

        std::shared_ptr<GeometricData> gd = leaf->getData()->getGeometricData();
        gd->updateBoundingBox();
        BoundingBox bb2 = gd->getBoundingBox();

        double scaleValue = targetWidth / bb2.size()(0) * 3;
        gd->transform(Eigen::Affine3d(Eigen::Scaling(scaleValue)));
        gd->transform(transform);
        gd->update();

        // set color
        leaf->getData()->getRenderModel()->setAppearances(
                    std::make_shared<Appearances>(
                        Appearance::createAppearanceFromColor(color)));

        // Source
        MeshCriteria criteria(0.0, 0.0, 0.02, 0.0, 0.0, false);
        SGLeafNode* source = mAc->getSGControl()->create3DGeometryFrom2D(
                    node->getName() + " (converted)",
                    parent,
                    leaf,
                    criteria,
                    true);

        // Set wireframe and invisible
        source->getData()->getRenderModel()->setWireframeEnabled(true);
        source->getData()->getRenderModel()->setVisible(false);

        if (rigid)
        {
            mAc->getSGControl()->createRigidBody(source->getData(), 1.0);
        }
        else
        {
            // Add to FEM simulation
            mAc->getSGControl()->createFEMObject(source->getData());
            std::shared_ptr<FEMObject> femObj =
                    std::dynamic_pointer_cast<FEMObject>(
                        source->getData()->getSimulationObject());
            femObj->setYoungsModulus(5e+2);
        }

        // Make collidable
        mAc->getSGControl()->createCollidable(source->getData(), 0.01);

        // Interpolator
        mAc->getInterpolatorModule()->addInterpolator(
                    source, leaf, MeshInterpolator::Type::FEM);

        return leaf;
    }

    return nullptr;
}
