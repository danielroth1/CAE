#include "Polygon3DTopology.h"

#include <map>
#include <set>

Polygon3DTopology::Polygon3DTopology(
        const Faces& faces,
        const Faces& outerFaces, // you can not call this constructor with already 2d outer faces
        const Cells& cells,
        ID nVertices)
    : PolygonTopology (faces, nVertices)
    , mOuterVertexIds(calculateOuterVertexIDs(outerFaces))
    , mOuterFacesIndices3D(outerFaces)
    , mCells(cells)
    , mOuterTopology(transformTo2DIndices(outerFaces, mOuterVertexIds),
                     mOuterVertexIds.size())
{

}

Cells& Polygon3DTopology::getCells()
{
    return mCells;
}

Polygon2DTopology& Polygon3DTopology::getOuterTopology()
{
    return mOuterTopology;
}

const Polygon2DTopology& Polygon3DTopology::getOuterTopology() const
{
    return mOuterTopology;
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

Faces& Polygon3DTopology::getOuterFacesIndices()
{
    return mOuterTopology.getFacesIndices();
}

const Faces& Polygon3DTopology::getOuterFacesIndices() const
{
    return mOuterTopology.getFacesIndices();
}

Faces& Polygon3DTopology::getOuterFacesIndices3D()
{
    return mOuterFacesIndices3D;
}

const Faces& Polygon3DTopology::getOuterFacesIndices3D() const
{
    return mOuterFacesIndices3D;
}

Edges Polygon3DTopology::retrieveOuterEdges() const
{
    return mOuterTopology.retrieveEdges();
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

Faces Polygon3DTopology::transformTo2DIndices(
        const Faces& faces,
        const std::vector<unsigned int>& outerVertexIds) const
{
    std::map<unsigned int, unsigned int> reverseMap;

    for (unsigned int i = 0; i < outerVertexIds.size(); ++i)
    {
        reverseMap[outerVertexIds[i]] = i;
    }

    Faces facesOut;
    facesOut.resize(faces.size());
    for (size_t i = 0; i < faces.size(); ++i)
    {
        for (size_t j = 0; j < 3; ++j)
        {
            facesOut[i][j] = reverseMap[faces[i][j]];
        }
    }

    return facesOut;
}
