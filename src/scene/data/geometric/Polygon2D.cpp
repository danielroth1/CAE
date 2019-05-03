#include "Polygon2D.h"

#include "GeometricDataUtils.h"
#include "Polygon2DDataBS.h"
#include "Polygon2DDataWS.h"

#include <scene/data/GeometricDataVisitor.h>

#include <scene/model/ModelUtils.h>

using namespace Eigen;

Polygon2D::Polygon2D(
        const Vectors& positionsWS,
        const Faces& faces)
    : Polygon(positionsWS)
{
    // World space construtor
    Polygon::initWorldSpace(positionsWS);
    mData = std::make_shared<Polygon2DDataWS>(faces, positionsWS.size());

    Vectors vertexNormals = GeometricDataUtils::calculateNormals(
                mPositionData.getPositions(), faces);
    mVertexNormals.initializeFromWorldSpace(vertexNormals);

    Vectors faceNormals;
    ModelUtils::calculateFaceNormals<double>(
                mPositionData.getPositions(), faces, faceNormals);
    mFaceNormals.initializeFromWorldSpace(faceNormals);
}

Polygon2D::Polygon2D(
        const Vectors& positionsWS,
        const Vectors& vertexNormalsWS,
        const Faces& faces)
    : Polygon(positionsWS)
{
    // World space constructor
    Polygon::initWorldSpace(positionsWS);
    mData = std::make_shared<Polygon2DDataWS>(faces, positionsWS.size());
    mVertexNormals.initializeFromWorldSpace(vertexNormalsWS);

    Vectors faceNormals;
    ModelUtils::calculateFaceNormals<double>(
                mPositionData.getPositions(), faces, faceNormals);
    mFaceNormals.initializeFromWorldSpace(faceNormals);
}

Polygon2D::Polygon2D(
        const Vectors& positionsBS,
        const Affine3d& transform,
        const Faces& faces)
    : Polygon()
{
    // Body space construtor

    Vectors vertexNormals = GeometricDataUtils::calculateNormals(
                mPositionData.getPositions(), faces);
    Vectors faceNormals = Vectors();
    std::shared_ptr<Polygon2DDataBS> dataBS =
            std::make_shared<Polygon2DDataBS>(
                faces,
                positionsBS,
                vertexNormals,
                faceNormals);

    mData = dataBS;
    Polygon::initBodySpace(&dataBS->getPositionsBS(), transform);
    mVertexNormals.initializeFromBodySpace(&dataBS->getVertexNormalsBS(),
                                           Eigen::Affine3d(transform.linear()));
    mFaceNormals.initializeFromBodySpace(&dataBS->getFaceNormalsBS(),
                                           Eigen::Affine3d(transform.linear()));
}

Polygon2D::Polygon2D(
        const Vectors& positionsBS,
        const Eigen::Affine3d& transform,
        const Vectors& vertexNormalsBS,
        const Faces& faces)
    : Polygon()
    , mFaceNormals(faces.size())
{
    // Body space constructor
    std::shared_ptr<Polygon2DDataBS> dataBS =
            std::make_shared<Polygon2DDataBS>(
                faces,
                positionsBS,
                vertexNormalsBS,
                Vectors());

    mData = dataBS;
    Polygon::initBodySpace(&dataBS->getPositionsBS(), transform);
    mVertexNormals.initializeFromBodySpace(&dataBS->getVertexNormalsBS(),
                                           Eigen::Affine3d(transform.linear()));
    mFaceNormals.initializeFromBodySpace(&dataBS->getFaceNormalsBS(),
                                         Eigen::Affine3d(transform.linear()));
}

Polygon2DTopology& Polygon2D::getTopology()
{
    return mData->getTopology();
}

const Polygon2DTopology& Polygon2D::getTopology() const
{
    return mData->getTopology();
}

std::shared_ptr<Polygon2DData> Polygon2D::getData2D()
{
    return mData;
}

std::shared_ptr<PolygonData> Polygon2D::getData()
{
    return mData;
}

void Polygon2D::update()
{
    Polygon::update();

    mVertexNormals.update();
    mFaceNormals.update();
}

void Polygon2D::accept(GeometricDataVisitor& visitor)
{
    visitor.visit(*this);
}

void Polygon2D::updateBoundingBox()
{
    // TODO: only when mPositionsWS is legal
    GeometricDataUtils::updateBoundingBox(
                mPositionData.getPositions(),
                mBoundingBox);
}

Polygon::Type Polygon2D::getType()
{
    return Type::TWO_D;
}

//void Polygon2D::changeRepresentationToBS(
//        Vectors* vectorsBS,
//        const Affine3d& transform)
//{
//    Polygon::changeRepresentationToBS(vectorsBS, transform);
//    mVertexNormals.changeRepresentationToBS();
//    mFaceNormals.changeRepresentationToBS();
//}

void Polygon2D::changeRepresentationToBS(const Eigen::Vector& center)
{
    std::shared_ptr<Polygon2DDataWS> dataWS =
            std::dynamic_pointer_cast<Polygon2DDataWS>(mData);
    if (dataWS)
    {
        std::shared_ptr<Polygon2DDataBS> dataBS =
                std::make_shared<Polygon2DDataBS>(mData->getTopology(),
                                                  mPositionData.getPositions(),
                                                  mVertexNormals.getVectors(),
                                                  mFaceNormals.getVectors());
        mData = dataBS;

        mPositionData.changeRepresentationToBS(
                    &dataBS->getPositionsBS(), Eigen::Affine3d::Identity());
        mPositionData.moveCenterTo(center);

        mVertexNormals.changeRepresentationToBS(
                    &dataBS->getVertexNormalsBS(), Eigen::Affine3d::Identity());
        mFaceNormals.changeRepresentationToBS(
                    &dataBS->getFaceNormalsBS(), Eigen::Affine3d::Identity());
    }
}

void Polygon2D::changeRepresentationToWS()
{
    std::shared_ptr<Polygon2DDataBS> dataBS =
            std::dynamic_pointer_cast<Polygon2DDataBS>(mData);
    if (dataBS)
    {
        std::shared_ptr<Polygon2DDataWS> dataWS =
                std::make_shared<Polygon2DDataWS>(mData->getTopology());
        mData = dataWS;

        mPositionData.changeRepresentationToWS();
        mVertexNormals.changeRepresentationToWS();
        mFaceNormals.changeRepresentationToWS();
    }
}

void Polygon2D::setTransform(const Affine3d& transform)
{
    Polygon::setTransform(transform);

    mVertexNormals.setTransform(Affine3d(transform.rotation()));
    mFaceNormals.setTransform(Affine3d(transform.rotation()));
}

Vectors& Polygon2D::getVertexNormals()
{
    return mVertexNormals.getVectors();
}

Vectors& Polygon2D::getFaceNormals()
{
    return mFaceNormals.getVectors();
}

