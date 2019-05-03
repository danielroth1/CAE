#include "Polygon3DTopology.h"

#include <set>

Polygon3DTopology::Polygon3DTopology(
        const Faces& faces,
        const Faces& outerFaces,
        const Cells& cells,
        ID nVertices)
    : PolygonTopology (faces, nVertices)
    , mOuterVertexIds(calculateOuterVertexIDs(outerFaces))
    , mFaces(faces)
    , mCells(cells)
    , mOuterTopology(outerFaces, nVertices) //mOuterVertexIds.size()) TODO topology
{

}

Cells& Polygon3DTopology::getCells()
{
    return mCells;
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

std::vector<TopologyEdge>& Polygon3DTopology::getOuterEdges()
{
    return mOuterTopology.getEdges();
}

const std::vector<TopologyEdge>& Polygon3DTopology::getOuterEdges() const
{
    return mOuterTopology.getEdges();
}

std::vector<TopologyFace>& Polygon3DTopology::getOuterFaces()
{
    return mOuterTopology.getFaces();
}

const std::vector<TopologyFace>& Polygon3DTopology::getOuterFaces() const
{
    return mOuterTopology.getFaces();
}

Edges Polygon3DTopology::retrieveOuterEdges() const
{
    return mOuterTopology.retrieveEdges();
}

Faces Polygon3DTopology::retrieveOuterFaces() const
{
    return mOuterTopology.retrieveFaces();
}

std::vector<unsigned int> Polygon3DTopology::calculateOuterVertexIDs(const Faces& faces)
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
