#include "InterpolationMeshMeshDemo.h"

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
#include <scene/data/geometric/MeshInterpolatorMeshMesh.h>
#include <scene/data/geometric/Polygon2D.h>
#include <scene/data/geometric/Polygon3D.h>
#include <scene/model/MeshInterpolatorRenderModel.h>
#include <scene/model/PolygonRenderModel.h>
#include <ui/UIControl.h>

using namespace Eigen;

InterpolationMeshMeshDemo::InterpolationMeshMeshDemo(ApplicationControl* ac)
    : mAc(ac)
{

}

std::string InterpolationMeshMeshDemo::getName()
{
    return "Interpolation Mesh Mesh";
}

void InterpolationMeshMeshDemo::load()
{
    mAc->getSimulationControl()->setGravity(Vector::Zero());
    mAc->getSimulationControl()->setNumFEMCorrectionIterations(0);
//            mAc->getSimulationControl()->setStepSize(0.001);

//            SGLeafNode* node2 = mAc->getSGControl()->createBox("Floor", mAc->getSGControl()->getSceneGraph()->getRoot(),
//                                                      /*Vector(-5, -2, -5)*/Vector(0.0, -1.5, 0.0), 2, 2, 2, true);
//            mAc->getSGControl()->createRigidBody(node2->getData(), 1.0, true);
//            mAc->getSGControl()->createCollidable(node2->getData(), 1.0);

//            SGLeafNode* node3 = mAc->getSGControl()->createBox("Floor", mAc->getSGControl()->getSceneGraph()->getRoot(),
//                                                      /*Vector(-5, -2, -5)*/Vector(0.0, -0.5, 0.0), 2, 2, 2, true);
//            mAc->getSGControl()->createRigidBody(node3->getData(), 1.0, true);
//            mAc->getSGControl()->createCollidable(node3->getData(), 1.0);


//            SGLeafNode* node2 = mAc->getSGControl()->createBox("Floor", mAc->getSGControl()->getSceneGraph()->getRoot(),
//                                                      /*Vector(-5, -2, -5)*/Vector(0.0, -1.5, 0.0), 2.5, 2.5, 2.5, true);
//            mAc->getSGControl()->createRigidBody(node2->getData(), 1.0, true);
//            mAc->getSGControl()->createCollidable(node2->getData(), 1.0);

//            SGLeafNode* node3 = mAc->getSGControl()->createBox("Floor", mAc->getSGControl()->getSceneGraph()->getRoot(),
//                                                      /*Vector(-5, -2, -5)*/Vector(0.0, -0.5, 0.0), 2, 2, 2, true);
//            mAc->getSGControl()->createRigidBody(node3->getData(), 1.0, true);
//            mAc->getSGControl()->createCollidable(node3->getData(), 1.0);

    bool useImportSource = true; // either use liberty statue or box as source
    bool addImportTarget = true; // loads liberty statue as target
    bool addSphere = false; // adds sphere as target
    bool interpolate = true;

    // some boxes:
    double boxDim = 0.5;
//    double boxDim = 6.0;

    std::vector<SGLeafNode*> targets;

    // load targets
    if (addImportTarget)
    {
        SGLeafNode* node2;
        OBJImporter importer;
        node2 = static_cast<SGLeafNode*>(importer.importFile(
                    File("/home/daniel/objs/LibertyStatue/LibertStatue.obj"),
                    mAc));
        mAc->getSGControl()->getSceneGraph()->getRoot()->addChild(node2);

//        std::shared_ptr<Polygon2D> libStatue =
//                std::dynamic_pointer_cast<Polygon2D>(
//                    node2->getData()->getGeometricData());

        // move to mid
//        std::shared_ptr<Polygon> poly2 =
//                std::static_pointer_cast<Polygon>(node2->getData()->getGeometricData());

//        Eigen::Vector mid = Eigen::Vector::Zero();
//        for (size_t i = 0; i < poly2->getPositions().size(); ++i)
//            mid += poly2->getPositions()[i];
//        mid /= poly2->getPositions().size();

//        Eigen::Affine3d scaling =
//                Eigen::Scaling(1.0) *
//                Eigen::Translation3d(-mid);
//        poly2->transform(scaling);

        targets.push_back(node2);

    }
    if (addSphere)
    {
        double sphereDim = 0.3;
        SGLeafNode* node = mAc->getSGControl()->createLeafNode(
                    "Box (detailed)",
                    mAc->getSGControl()->getSceneGraph()->getRoot(),
                    std::make_shared<Polygon2D>(
                        GeometricDataFactory::create2DSphere(sphereDim, 4)),
                    Vector(0.0, -0.05, 0.0), true);
//                    Vector(0.0, 0.0, 0.0), true);

        // scale down a bit
        std::shared_ptr<Polygon> poly =
                std::static_pointer_cast<Polygon>(node->getData()->getGeometricData());

//        Eigen::Vector mid = Eigen::Vector::Zero();
//        for (size_t i = 0; i < poly->getPositions().size(); ++i)
//            mid += poly->getPositions()[i];
//        mid /= poly->getPositions().size();

//        Eigen::Affine3d scaling =
//                Eigen::Translation3d(mid) *
//                Eigen::Scaling(0.1) *
//                Eigen::Translation3d(-mid);
//        poly->transform(scaling);

//        node = mAc->getSGControl()->createBox(
//                    "Box (detailed)", mAc->getSGControl()->getSceneGraph()->getRoot(),
//                    Vector(0.0, 0.0, 0.0),
//                    boxDim*1.5, boxDim*1.5, boxDim*1.5, true);
//        MeshCriteria criteria(0.0, 0.0, 0.0, 0.15, 30, true);
//        mAc->getSGControl()->create3DGeometryFrom2D(node, criteria, true);

        targets.push_back(node);
    }

    // load the source
    SGLeafNode* sourceNode;
    if (!useImportSource)
    {
        // use a 3d box
        sourceNode = mAc->getSGControl()->createLeafNode(
                    "Box", mAc->getSGControl()->getSceneGraph()->getRoot(),
                    std::make_shared<Polygon3D>(
                        GeometricDataFactory::create3DBox(boxDim, boxDim, boxDim)),
                    Vector(0.0, 0.0, 0.0), true);

        // scale down a bit
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
    }
    else
    {
        // import liberty statue and apply mesh converter.
        // Scale down the resulting model
        OBJImporter importer;
        sourceNode = static_cast<SGLeafNode*>(importer.importFile(
                    File("/home/daniel/objs/LibertyStatue/LibertStatue.obj"),
                    mAc));
        mAc->getSGControl()->getSceneGraph()->getRoot()->addChild(sourceNode);
        MeshCriteria criteria(0.0, 0.0, 0.0, 0.1, 20, false);
        mAc->getSGControl()->create3DGeometryFrom2D(sourceNode, criteria, true);

        // scale down a bit
//        std::shared_ptr<Polygon> poly1 =
//                std::static_pointer_cast<Polygon>(sourceNode->getData()->getGeometricData());

//        Eigen::Vector mid = Eigen::Vector::Zero();
//        for (size_t i = 0; i < poly1->getPositions().size(); ++i)
//            mid += poly1->getPositions()[i];
//        mid /= poly1->getPositions().size();

////                        Eigen::Affine3d scaling =
////                                Eigen::Translation3d(mid) *
////                                Eigen::Scaling(2.0) *
////                                Eigen::Translation3d(-mid);

//        Eigen::Affine3d scaling =
//                Eigen::Scaling(0.5) *
//                Eigen::Translation3d(-mid);
//        poly1->transform(scaling);
    }

    // deformable
//                    MeshCriteria criteria(0.0, 0.0, 0.0, 0.15, 30, true);
//                    MeshCriteria criteria(0.0, 0.0, 0.0, 0.15, 30, false);
//                    mAc->getSGControl()->create3DGeometryFrom2D(sourceNode, criteria, true);

//    // move geometries away from origin
//    Eigen::Affine3d trans = Eigen::Scaling(10.0) *
//            Eigen::Translation3d(0.0, 0.0, 0.0);
//    for (SGLeafNode* target : targets)
//    {
//        std::static_pointer_cast<Polygon>(
//                    target->getData()->getGeometricData())->transform(trans);
//        target->getData()->getGeometricData()->geometricDataChanged();
//    }
//    std::static_pointer_cast<Polygon>(
//                sourceNode->getData()->getGeometricData())->transform(trans);
//    sourceNode->getData()->getGeometricData()->geometricDataChanged();

    if (interpolate)
    {
        // move geometries away from origin
//        Eigen::Affine3d trans = Eigen::Scaling(4.0) *
//                Eigen::Translation3d(0.0, 0.0, 0.0);
//        for (SGLeafNode* target : targets)
//        {
//            std::static_pointer_cast<Polygon>(
//                        target->getData()->getGeometricData())->transform(trans);
//            target->getData()->getGeometricData()->geometricDataChanged();
//        }
//        std::static_pointer_cast<Polygon>(
//                    sourceNode->getData()->getGeometricData())->transform(trans);
//        sourceNode->getData()->getGeometricData()->geometricDataChanged();

        for (SGLeafNode* target : targets)
        {
            addInterpolation(sourceNode, target);
        }

//        // move geometries away from origin
//        trans = Eigen::Scaling(0.25) *
//                Eigen::Translation3d(0.0, 0.0, 0.0);
//        std::static_pointer_cast<Polygon>(
//                    sourceNode->getData()->getGeometricData())->transform(trans);
//        sourceNode->getData()->getGeometricData()->geometricDataChanged();
    }

    mAc->getSGControl()->createFEMObject(sourceNode->getData());

    std::shared_ptr<PolygonRenderModel> renderModel =
            std::static_pointer_cast<PolygonRenderModel>(
                sourceNode->getData()->getRenderModel());

    // load some example image and set it as texture to render model
    std::shared_ptr<Texture> texture =
            std::make_shared<Texture>(
                ImageLoader::instance()->loadBMP(
                    "/home/daniel/objs/LibertyStatue/Liberty-PortaBronzo-1.bmp"));

    std::shared_ptr<Appearances> appearances =
            std::make_shared<Appearances>(
                std::make_shared<Appearance>(texture));

    renderModel->setAppearances(appearances);

    std::shared_ptr<Polygon> poly =
            std::dynamic_pointer_cast<Polygon>(
                sourceNode->getData()->getGeometricData());

    std::vector<Eigen::Vector2f> textureCoordinates =
            TextureUtils::createSpericalTextureCoordinates<float, double>(
                poly->getPositions());

    std::cout << "texture coordinates:\n";
    for (size_t i = 0; i < textureCoordinates.size(); ++i)
    {
        std::cout << textureCoordinates[i].transpose() << " -> " <<
                     poly->getPosition(i).transpose() << "\n";
    }

    renderModel->setTextureCoordinates(textureCoordinates);
    renderModel->setTexturingEnabled(true);
}

void InterpolationMeshMeshDemo::addInterpolation(
            SGLeafNode* sourceNode,
            SGLeafNode* targetNode)
{
    mInterpolator = std::make_shared<MeshInterpolatorMeshMesh>(
                std::dynamic_pointer_cast<Polygon>(
                    sourceNode->getData()->getGeometricData()),
                std::dynamic_pointer_cast<Polygon>(
                    targetNode->getData()->getGeometricData()));
    mInterpolator->solveNewton();

    mInterpolatorModel = std::make_shared<MeshInterpolatorRenderModel>(
                mInterpolator, false, false);

    Renderer* renderer = mAc->getUIControl()->getRenderer();
    mInterpolatorModel->addToRenderer(renderer);
    mInterpolatorModel->setAddedToRenderer(true);

    class mInterpolatorModelUpdater : public GeometricDataListener
    {
    public:
        mInterpolatorModelUpdater(
                    const std::shared_ptr<MeshInterpolator>& _mInterpolatorModel,
                    const std::shared_ptr<MeshInterpolatorRenderModel>& _renderModel)
            : mInterpolatorModel(_mInterpolatorModel)
            , renderModel(_renderModel)
        {

        }

        virtual void notifyGeometricDataChanged()
        {
            mInterpolatorModel->update();
            renderModel->update();
        }

        std::shared_ptr<MeshInterpolator> mInterpolatorModel;
        std::shared_ptr<MeshInterpolatorRenderModel> renderModel;
    };

    std::shared_ptr<mInterpolatorModelUpdater> updater =
            std::make_shared<mInterpolatorModelUpdater>(
                mInterpolator,
                mInterpolatorModel);

    sourceNode->getData()->getGeometricData()->addGeometricDataListener(
                updater);
}

void InterpolationMeshMeshDemo::unload()
{

}
