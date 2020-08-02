#include "InterpolationFEMDemo.h"

#include "ApplicationControl.h"
#include <QCoreApplication>
#include <data_structures/DataStructures.h>
#include <io/ImageLoader.h>
#include <io/importers/OBJImporter.h>
#include <modules/interpolator/InterpolatorModule.h>
#include <modules/mesh_converter/MeshCriteria.h>
#include <rendering/Appearance.h>
#include <rendering/Appearances.h>
#include <rendering/Renderer.h>
#include <rendering/Texture.h>
#include <rendering/TextureUtils.h>
#include <scene/data/geometric/GeometricDataFactory.h>
#include <scene/data/geometric/GeometricDataListener.h>
#include <scene/data/geometric/MeshInterpolatorFEM.h>
#include <scene/data/geometric/Polygon2D.h>
#include <scene/data/geometric/Polygon3D.h>
#include <scene/model/MeshInterpolatorRenderModel.h>
#include <scene/model/PolygonRenderModel.h>
#include <simulation/fem/FEMObject.h>
#include <ui/UIControl.h>

using namespace Eigen;

InterpolationFEMDemo::InterpolationFEMDemo(
        ApplicationControl* ac,
        bool simpleMeshes,
        MeshInterpolator::Type interpolatorType)
    : mAc(ac)
    , mInterpolatorType(interpolatorType)
    , mSimpleMeshes(simpleMeshes)
{

}

std::string InterpolationFEMDemo::getName()
{
    std::string name;
    if (mInterpolatorType == MeshInterpolator::Type::FEM)
        name = "Interpolation FEM";
    else
        name = "Interpolation MeshMesh";

    if (mSimpleMeshes)
        name += " (Cube)";
    else
        name += " (Astronaut)";

    return name;
}

void InterpolationFEMDemo::load()
{
//    mAc->getSimulationControl()->setNumFEMCorrectionIterations(0);

    bool useImportSource = true; // either use nasa suit or box as source
    bool addImportTarget = true; // loads nasa suit as target
    bool addSphere = false; // adds sphere as target
    bool interpolate = true;

    if (mSimpleMeshes)
    {
        useImportSource = false;
        addImportTarget = false;
        addSphere = true;
    }

    Eigen::Affine3d complexMeshTransform =
            Eigen::Translation3d(-1, 0, 8) *
            Eigen::AngleAxisd(180.0 * M_PI / 180.0, Eigen::Vector3d(0.0, 1.0, 0.0));

    // some boxes:
    double boxDim = 0.8;

    std::vector<SGLeafNode*> targets;

    // load targets
    if (addImportTarget)
    {
        SGLeafNode* node2;
        OBJImporter importer;
        node2 = static_cast<SGLeafNode*>(importer.importFile(
                    File(QCoreApplication::applicationDirPath().toStdString() + "/assets/nasa/advanced_crew_escape_suit.obj"),
                    mAc));
        mAc->getSGControl()->getSceneGraph()->getRoot()->addChild(node2);
        node2->getData()->setVerticesSelectable(false);

        node2->getData()->getGeometricData()->transform(complexMeshTransform);

        targets.push_back(node2);

    }
    if (addSphere)
    {
        double sphereDim = 0.4;
        SGLeafNode* node = mAc->getSGControl()->createLeafNode(
                    "Sphere (detailed)",
                    mAc->getSGControl()->getSceneGraph()->getRoot(),
                    std::make_shared<Polygon2D>(
                        GeometricDataFactory::create2DSphere(sphereDim, 4)),
                    Vector(0.0, 0.0, 0.0), true);

        node->getData()->getRenderModel()->setWireframeEnabled(true);

        targets.push_back(node);

        mAc->getSimulationControl()->setGravity(Vector::Zero());
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
        // import nasa suit and apply mesh converter.
        // Scale down the resulting model
        OBJImporter importer;
        sourceNode = static_cast<SGLeafNode*>(importer.importFile(
                    File(QCoreApplication::applicationDirPath().toStdString() + "/assets/nasa/advanced_crew_escape_suit_convex.obj"),
                    mAc));
        mAc->getSGControl()->getSceneGraph()->getRoot()->addChild(sourceNode);
        MeshCriteria criteria(0.0, 0.0, 0.0, 0.1, 20, false);
        mAc->getSGControl()->create3DGeometryFrom2D(sourceNode, criteria, true);
        sourceNode->getData()->getGeometricData()->transform(complexMeshTransform);
    }

    SGLeafNode* node = static_cast<SGLeafNode*>(mAc->getSGControl()->importFileAsChild(
                File(QCoreApplication::applicationDirPath().toStdString() + "/assets/fractal_terrain.obj"),
                mAc->getSGControl()->getSceneGraph()->getRoot()));
    node->getData()->setVerticesSelectable(false);
    mAc->getSGControl()->createRigidBody(node->getData(), 1, true);
    mAc->getSGControl()->createCollidable(node->getData());

    sourceNode->getData()->getRenderModel()->setWireframeEnabled(true);

    if (interpolate)
    {
        for (SGLeafNode* target : targets)
        {
            mAc->getInterpolatorModule()->addInterpolator(
                        sourceNode, target, mInterpolatorType);
            mAc->getInterpolatorModule()->setInterpolatorVisible(
                        std::dynamic_pointer_cast<Polygon>(
                            target->getData()->getGeometricData()), true);
        }
    }

    std::shared_ptr<FEMObject> femObj =
            mAc->getSGControl()->createFEMObject(sourceNode->getData());
    mAc->getSGControl()->createCollidable(sourceNode->getData());
    femObj->setYoungsModulus(500);
}

void InterpolationFEMDemo::unload()
{

}
