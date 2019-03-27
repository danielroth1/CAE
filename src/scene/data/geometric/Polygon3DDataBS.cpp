#include "Polygon3DDataBS.h"
#include "Polygon3DTopology.h"


Polygon3DDataBS::Polygon3DDataBS(
        const std::vector<unsigned int>& outerVertexIds,
        const Edges& edges,
        const Edges& outerEdges,
        const Faces& faces,
        const Faces& outerFaces,
        const Cells& cells,
        const Vectors& positionsBS,
        const Vectors& outerVertexNormalsBS,
        const Vectors& outerFaceNormalsBS)
    : Polygon3DData (outerVertexIds,
                     edges,
                     outerEdges,
                     faces,
                     outerFaces,
                     cells)
    , mPositionsBS(positionsBS)
    , mOuterVertexNormalsBS(outerVertexNormalsBS)
    , mOuterFaceNormalsBS(outerFaceNormalsBS)
{
}

Polygon3DDataBS::~Polygon3DDataBS()
{

}

Vectors& Polygon3DDataBS::getPositionsBS()
{
    return mPositionsBS;
}

void Polygon3DDataBS::setPositionsBS(const Vectors& positionsBS)
{
    mPositionsBS = positionsBS;
}

Vectors& Polygon3DDataBS::getOuterVertexNormalsBS()
{
    return mOuterVertexNormalsBS;
}

void Polygon3DDataBS::setOuterVertexNormalsBS(const Vectors& outerVertexNormalsBS)
{
    mOuterVertexNormalsBS = outerVertexNormalsBS;
}

Vectors& Polygon3DDataBS::getOuterFaceNormalsBS()
{
    return mOuterFaceNormalsBS;
}

void Polygon3DDataBS::setOuterFaceNormalsBS(const Vectors& outerFaceNormalsBS)
{
    mOuterFaceNormalsBS = outerFaceNormalsBS;
}
