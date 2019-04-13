#include "Polygon3D.h"

#include "GeometricDataUtils.h"
#include "Polygon3DDataBS.h"
#include "Polygon3DDataWS.h"

#include <scene/data/GeometricDataVisitor.h>
#include <set>

Polygon3D::Polygon3D(
        const Vectors& positionsWS,
        const Faces& faces,
        const Faces& outerFaces,
        const Cells& cells)
    : Polygon(positionsWS)
{
    mData = std::make_shared<Polygon3DDataWS>(
                calculateOuterPositionIDs(faces),
                calculateEdges(faces),
                calculateEdges(outerFaces),
                faces,
                outerFaces,
                cells);

    mOuterVertexNormals.initializeFromWorldSpace(
                GeometricDataUtils::calculateNormals(positionsWS, outerFaces));

    // TODO: initialize mOuterFaceNormals
}

Polygon3D::Polygon3D(
        const Vectors& positionsWS,
        const Vectors& vertexNormalsWS,
        const Faces& faces,
        const Faces& outerFaces,
        const Cells& cells)
    : Polygon(positionsWS)
{
    mData = std::make_shared<Polygon3DDataWS>(
                calculateOuterPositionIDs(faces),
                calculateEdges(faces),
                calculateEdges(outerFaces),
                faces,
                outerFaces,
                cells);

    mOuterVertexNormals.initializeFromWorldSpace(vertexNormalsWS);

    // TODO: initialize mOuterFaceNormals
}

Polygon3D::Polygon3D(
        Vectors* positionsBS,
        const Affine3d& transform,
        const Faces& faces,
        const Faces& outerFaces,
        const Cells& cells)
    : Polygon(positionsBS, transform)
{
    std::shared_ptr<Polygon3DDataBS> dataBS = std::make_shared<Polygon3DDataBS>(
                calculateOuterPositionIDs(faces),
                calculateEdges(faces),
                calculateEdges(outerFaces),
                faces,
                outerFaces,
                cells,
                mPositionData.getPositionsBS(),
                GeometricDataUtils::calculateNormals(*positionsBS, outerFaces),
                Vectors());

    mData = dataBS;
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
    std::shared_ptr<Polygon3DDataBS> dataBS = std::make_shared<Polygon3DDataBS>(
                calculateOuterPositionIDs(faces),
                calculateEdges(faces),
                calculateEdges(outerFaces),
                faces,
                outerFaces,
                cells,
                mPositionData.getPositionsBS(),
                vertexNormalsBS,
                Vectors());

    mData = dataBS;
    mOuterVertexNormals.initializeFromBodySpace(&dataBS->getOuterVertexNormalsBS(),
                                                Eigen::Affine3d(transform.linear()));
    mOuterFaceNormals.initializeFromBodySpace(&dataBS->getOuterFaceNormalsBS(),
                                              Eigen::Affine3d(transform.linear()));
}

Polygon3D::~Polygon3D()
{

}

Edges& Polygon3D::getEdges()
{
    return mData->getEdges();
}

Edges& Polygon3D::getOuterEdges()
{
    return mData->getOuterEdges();
}

Faces& Polygon3D::getOuterFaces()
{
    return mData->getOuterFaces();
}

Faces& Polygon3D::getFaces()
{
    return mData->getFaces();
}

Cells& Polygon3D::getCells()
{
    return mData->getCells();
}

std::vector<unsigned int>& Polygon3D::getOuterPositionIds()
{
    return mData->getOuterVertexIds();
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
                    mData->getOuterVertexIds(),
                    mData->getEdges(),
                    mData->getOuterEdges(),
                    mData->getFaces(),
                    mData->getOuterFaces(),
                    mData->getCells(),
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
                    mData->getOuterVertexIds(),
                    mData->getEdges(),
                    mData->getOuterEdges(),
                    mData->getFaces(),
                    mData->getOuterFaces(),
                    mData->getCells());
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

std::vector<unsigned int> Polygon3D::calculateOuterPositionIDs(const Faces& faces)
{
    std::vector<unsigned int> outerPositionIds;

    std::set<unsigned int> idsSet;
    for (const Face& face : faces)
    {
        idsSet.insert(face[0]);
        idsSet.insert(face[1]);
        idsSet.insert(face[2]);
    }

    outerPositionIds.reserve(idsSet.size());
    for (unsigned int id : idsSet)
    {
        outerPositionIds.push_back(id);
    }

//    std::move(idsSet.begin(), idsSet.end(), outerPositionIds.begin());
    std::sort(outerPositionIds.begin(), outerPositionIds.end(), std::less<unsigned int>());

    return outerPositionIds;
}

