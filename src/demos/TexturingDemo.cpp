#include "TexturingDemo.h"

#include <ApplicationControl.h>
#include <QCoreApplication>
#include <data_structures/DataStructures.h>
#include <io/ImageLoader.h>
#include <rendering/Appearance.h>
#include <rendering/Appearances.h>
#include <rendering/Texture.h>
#include <rendering/TextureUtils.h>
#include <scene/data/geometric/GeometricDataFactory.h>
#include <scene/data/geometric/Polygon3D.h>
#include <scene/model/PolygonRenderModel.h>

using namespace Eigen;

TexturingDemo::TexturingDemo(ApplicationControl* ac)
    : mAc(ac)
{

}

std::string TexturingDemo::getName()
{
    return "Texturing Demo";
}

void TexturingDemo::load()
{
    mAc->getSimulationControl()->setGravity(Vector::Zero());
//    mAc->getSimulationControl()->setNumFEMCorrectionIterations(0);

    double boxDim = 3.0;

    SGLeafNode* node1 = mAc->getSGControl()->createLeafNode(
                "Box", mAc->getSGControl()->getSceneGraph()->getRoot(),
                std::make_shared<Polygon3D>(
                    GeometricDataFactory::create3DBox(boxDim, boxDim, boxDim)),
                Vector(0.0, 0.0, 0.0), true);

    // deformable
//                    MeshCriteria criteria(0.0, 0.0, 0.0, 0.15, 30, true);
//                    MeshCriteria criteria(0.0, 0.0, 0.0, 0.15, 30, false);
//                    ac.mSGControl->create3DGeometryFrom2D(node1, criteria, true);

    mAc->getSGControl()->createFEMObject(node1->getData());

    std::shared_ptr<PolygonRenderModel> renderModel =
            std::static_pointer_cast<PolygonRenderModel>(
                node1->getData()->getRenderModel());

    // load some example image and set it as texture to render model
    std::shared_ptr<Texture> texture =
            std::make_shared<Texture>(
                ImageLoader::instance()->loadBMP(
                    QCoreApplication::applicationDirPath().toStdString() + "/assets/textures/stonetiles_002_diff.bmp"));

    std::shared_ptr<Appearances> appearances =
            std::make_shared<Appearances>(
                std::make_shared<Appearance>(texture));

    renderModel->setAppearances(appearances);

    std::shared_ptr<Polygon> poly =
            std::dynamic_pointer_cast<Polygon>(
                node1->getData()->getGeometricData());

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

void TexturingDemo::unload()
{

}
