#include "Polygon3DTopology.h"

Polygon3DTopology::Polygon3DTopology(
        const std::vector<unsigned int>& outerVertexIds,
        const Edges& edges,
        const Edges& outerEdges,
        const Faces& faces,
        const Faces& outerFaces,
        const Cells& cells)
    : mOuterVertexIds(outerVertexIds)
    , mEdges(edges)
    , mOuterEdges(outerEdges)
    , mFaces(faces)
    , mOuterFaces(outerFaces)
    , mCells(cells)
{

}

std::vector<unsigned int>& Polygon3DTopology::getOuterVertexIds()
{
    return mOuterVertexIds;
}

void Polygon3DTopology::setOuterVertexIds(
        const std::vector<unsigned int>& outerVertexIds)
{
    mOuterVertexIds = outerVertexIds;
}

Edges& Polygon3DTopology::getEdges()
{
    return mEdges;
}

void Polygon3DTopology::setEdges(const Edges& edges)
{
    mEdges = edges;
}

Edges& Polygon3DTopology::getOuterEdges()
{
    return mOuterEdges;
}

void Polygon3DTopology::setOuterEdges(const Edges& outerEdges)
{
    mOuterEdges = outerEdges;
}

Faces& Polygon3DTopology::getFaces()
{
    return mFaces;
}

void Polygon3DTopology::setFaces(const Faces& faces)
{
    mFaces = faces;
}

Faces& Polygon3DTopology::getOuterFaces()
{
    return mOuterFaces;
}

void Polygon3DTopology::setOuterFaces(const Faces& outerFaces)
{
    mOuterFaces = outerFaces;
}

Cells& Polygon3DTopology::getCells()
{
    return mCells;
}

void Polygon3DTopology::setCells(const Cells& cells)
{
    mCells = cells;
}
