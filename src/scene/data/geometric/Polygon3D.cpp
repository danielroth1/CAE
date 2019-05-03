#include "Polygon3D.h"

#include "GeometricDataUtils.h"
#include "Polygon3DDataBS.h"
#include "Polygon3DDataWS.h"
#include "Polygon3DTopology.h"

#include <scene/data/GeometricDataVisitor.h>
#include <scene/model/ModelUtils.h>
#include <set>

Polygon3D::Polygon3D(
        const Vectors& positionsWS,
        const Faces& faces,
        const Faces& outerFaces,
        const Cells& cells)
    : Polygon(positionsWS)
{
    // World space construtor
    Polygon::initWorldSpace(positionsWS);
    mData = std::make_shared<Polygon3DDataWS>(
                faces,
                outerFaces,
                cells,
                positionsWS.size());

    mOuterVertexNormals.initializeFromWorldSpace(
                GeometricDataUtils::calculateNormals(positionsWS, outerFaces));

    // initialize mOuterFaceNormals
    Vectors faceNormals;
    ModelUtils::calculateFaceNormals<double>(
                mPositionData.getPositions(), faces, faceNormals);
    mOuterFaceNormals.initializeFromWorldSpace(faceNormals);
}

Polygon3D::Polygon3D(
        const Vectors& positionsWS,
        const Vectors& vertexNormalsWS,
        const Faces& faces,
        const Faces& outerFaces,
        const Cells& cells)
    : Polygon(positionsWS)
{
    // World space construtor
    Polygon::initWorldSpace(positionsWS);
    mData = std::make_shared<Polygon3DDataWS>(
                faces,
                outerFaces,
                cells,
                positionsWS.size());

    mOuterVertexNormals.initializeFromWorldSpace(vertexNormalsWS);

    // initialize mOuterFaceNormals
    Vectors faceNormals;
    ModelUtils::calculateFaceNormals<double>(
                mPositionData.getPositions(), faces, faceNormals);
    mOuterFaceNormals.initializeFromWorldSpace(faceNormals);
}

Polygon3D::Polygon3D(
        Vectors* positionsBS,
        const Affine3d& transform,
        const Faces& faces,
        const Faces& outerFaces,
        const Cells& cells)
    : Polygon(positionsBS, transform)
{
    Vectors faceNormals = GeometricDataUtils::calculateNormals(*positionsBS, outerFaces);
    std::shared_ptr<Polygon3DDataBS> dataBS = std::make_shared<Polygon3DDataBS>(
                faces,
                outerFaces,
                cells,
                mPositionData.getPositionsBS(),
                faceNormals,
                ModelUtils::calculateFaceNormals<double>(
                            mPositionData.getPositions(), faces, faceNormals));

    mData = dataBS;
    Polygon::initBodySpace(&dataBS->getPositionsBS(), transform);
    mOuterVertexNormals.initializeFromBodySpace(&dataBS->getOuterVertexNormalsBS(),
                                                Eigen::Affine3d(transform.linear()));
    mOuterFaceNormals.initializeFromBodySpace(&dataBS->getOuterFaceNormalsBS(),
                                              Eigen::Affine3d(transform.linear()));
}

Polygon3D::Polygon3D(
        Vectors* positionsBS,
        const Affine3d& transform,
        const Vectors& vertexNormalsBS,
        const Faces& faces,
        const Faces& outerFaces,
        const Cells& cells)
    : Polygon (positionsBS, transform)
{
    Vectors faceNormals = GeometricDataUtils::calculateNormals(*positionsBS, outerFaces);
    std::shared_ptr<Polygon3DDataBS> dataBS = std::make_shared<Polygon3DDataBS>(
                faces,
                outerFaces,
                cells,
                mPositionData.getPositionsBS(),
                vertexNormalsBS,
                ModelUtils::calculateFaceNormals<double>(
                            mPositionData.getPositions(), faces, faceNormals));

    mData = dataBS;
    Polygon::initBodySpace(&dataBS->getPositionsBS(), transform);
    mOuterVertexNormals.initializeFromBodySpace(&dataBS->getOuterVertexNormalsBS(),
                                                Eigen::Affine3d(transform.linear()));
    mOuterFaceNormals.initializeFromBodySpace(&dataBS->getOuterFaceNormalsBS(),
                                              Eigen::Affine3d(transform.linear()));
}

Polygon3D::~Polygon3D()
{

}

Polygon3DTopology& Polygon3D::getTopology()
{
    return mData->getTopology();
}

const Polygon3DTopology& Polygon3D::getTopology() const
{
    return mData->getTopology();
}

std::vector<unsigned int>& Polygon3D::getOuterPositionIds()
{
    return mData->getTopology().getOuterVertexIds();
}

std::shared_ptr<Polygon3DData> Polygon3D::getData3D()
{
    return mData;
}

void Polygon3D::updateBoundingBox()
{
    GeometricDataUtils::updateBoundingBox(
                mPositionData.getPositions(), mBoundingBox);
}

void Polygon3D::accept(GeometricDataVisitor& visitor)
{
    visitor.visit(*this);
}

void Polygon3D::update()
{
    Polygon::update();

    mOuterVertexNormals.update();
    mOuterFaceNormals.update();
}

Polygon::Type Polygon3D::getType()
{
    return Type::THREE_D;
}

std::shared_ptr<PolygonData> Polygon3D::getData()
{
    return mData;
}

void Polygon3D::changeRepresentationToBS(const Vector& center)
{
    std::shared_ptr<Polygon3DDataWS> dataWS =
            std::dynamic_pointer_cast<Polygon3DDataWS>(mData);
    if (dataWS)
    {
        std::shared_ptr<Polygon3DDataBS> dataBS =
                std::make_shared<Polygon3DDataBS>(
                    mData->getTopology().retrieveFaces(),
                    mData->getTopology().retrieveOuterFaces(),
                    mData->getTopology().getCells(),
                    mPositionData.getPositions(),
                    mOuterVertexNormals.getVectors(),
                    mOuterFaceNormals.getVectors());
        mData = dataBS;

        mPositionData.changeRepresentationToBS(
                    &dataBS->getPositionsBS(), Eigen::Affine3d::Identity());

        mPositionData.moveCenterTo(center);
        mOuterVertexNormals.changeRepresentationToBS(
                    &dataBS->getOuterVertexNormalsBS(), Eigen::Affine3d::Identity());
        mOuterFaceNormals.changeRepresentationToBS(
                    &dataBS->getOuterFaceNormalsBS(), Eigen::Affine3d::Identity());
    }
}

void Polygon3D::changeRepresentationToWS()
{
    std::shared_ptr<Polygon3DDataBS> dataBS =
            std::dynamic_pointer_cast<Polygon3DDataBS>(mData);
    if (dataBS)
    {
        std::shared_ptr<Polygon3DDataWS> dataWS =
                std::make_shared<Polygon3DDataWS>(
                    mData->getTopology().retrieveFaces(),
                    mData->getTopology().retrieveOuterFaces(),
                    mData->getTopology().getCells(),
                    dataBS->getPositionsBS().size());
        mData = dataWS;

        mPositionData.changeRepresentationToWS();
        mOuterVertexNormals.changeRepresentationToWS();
        mOuterFaceNormals.changeRepresentationToWS();
    }
}

void Polygon3D::setTransform(const Affine3d& transform)
{
    Polygon::setTransform(transform);

    mOuterVertexNormals.setTransform(Affine3d(transform.rotation()));
    mOuterFaceNormals.setTransform(Affine3d(transform.rotation()));
}
