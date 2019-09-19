#include "InterpolationFEMDemo.h"

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

using namespace Eigen;

InterpolationFEMDemo::InterpolationFEMDemo(ApplicationControl* ac)
    : mAc(ac)
{

}

std::string InterpolationFEMDemo::getName()
{
    return "Interpolation FEM";
}

void InterpolationFEMDemo::load()
{
    mAc->getSimulationControl()->setGravity(Vector::Zero());
    mAc->getSimulationControl()->setNumFEMCorrectionIterations(0);

    bool useImportSource = false; // either use liberty statue or box as source
    bool addImportTarget = false; // loads liberty statue as target
    bool addSphere = true; // adds sphere as target
    bool interpolate = true;

    // some boxes:
    double boxDim = 0.8;

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
                    Vector(0.0, 0.0, 0.0), true);

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
    }

    sourceNode->getData()->getRenderModel()->setWireframeEnabled(true);

    if (interpolate)
    {
        for (SGLeafNode* target : targets)
        {
            addInterpolation(sourceNode, target);
            mAc->getMeshInterpolationManager()->setInterpolatorVisible(
                        std::dynamic_pointer_cast<Polygon>(
                            target->getData()->getGeometricData()), true);
        }
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

void InterpolationFEMDemo::addInterpolation(
            SGLeafNode* sourceNode,
            SGLeafNode* targetNode)
{
    mAc->getMeshInterpolationManager()->addInterpolatorFEM(
                std::dynamic_pointer_cast<Polygon3D>(
                    sourceNode->getData()->getGeometricData()),
                std::dynamic_pointer_cast<Polygon>(
                    targetNode->getData()->getGeometricData()));
}

void InterpolationFEMDemo::unload()
{

}
