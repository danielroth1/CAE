#include "Polygon2DDataBS.h"

#include <data_structures/VectorOperations.h>


Polygon2DDataBS::Polygon2DDataBS(
        const Polygon2DTopology& topology,
        const Vectors& positionsBS,
        const Vectors& vertexNormalsBS,
        const Vectors& faceNormalsBS)
    : Polygon2DData(topology)
    , mPositionsBS(positionsBS)
    , mVertexNormalsBS(vertexNormalsBS)
    , mFacesNormalsBS(faceNormalsBS)
{

}

Polygon2DDataBS::~Polygon2DDataBS()
{

}

void Polygon2DDataBS::setPositionsBS(const Vectors& positionsBS)
{
    mPositionsBS = positionsBS;
}

void Polygon2DDataBS::setVertexNormalsBS(const Vectors& vertexNormalsBS)
{
    mVertexNormalsBS = vertexNormalsBS;
}

void Polygon2DDataBS::setFaceNormalsBS(const Vectors& faceNormalsBS)
{
    mFacesNormalsBS = faceNormalsBS;
}

Vectors& Polygon2DDataBS::getPositionsBS()
{
    return mPositionsBS;
}

Vectors& Polygon2DDataBS::getVertexNormalsBS()
{
    return mVertexNormalsBS;
}

Vectors& Polygon2DDataBS::getFaceNormalsBS()
{
    return mFacesNormalsBS;
}

BSWSVectors::Type Polygon2DDataBS::getPositionType()
{
    return BSWSVectors::Type::BODY_SPACE;
}

void Polygon2DDataBS::removeVector(ID index)
{
    VectorOperations::removeVector(mPositionsBS, index);
    VectorOperations::removeVector(mVertexNormalsBS, index);
    VectorOperations::removeVector(mFacesNormalsBS, index);
}

void Polygon2DDataBS::removeVectors(std::vector<ID>& indices)
{
    VectorOperations::removeVectors(mPositionsBS, indices.begin(), indices.end());
    VectorOperations::removeVectors(mVertexNormalsBS, indices.begin(), indices.end());
    VectorOperations::removeVectors(mFacesNormalsBS, indices.begin(), indices.end());
}
