#include "TopologyEdge.h"


TopologyEdge::TopologyEdge(ID id)
    : TopologyFeature(id)
{
    mVertices.reserve(2);
    mFaces.reserve(2);
}

std::vector<ID>& TopologyEdge::getVertexIds()
{
    return mVertices;
}

const std::vector<ID>& TopologyEdge::getVertexIds() const
{
    return mVertices;
}

std::vector<ID>& TopologyEdge::getFaceIds()
{
    return mFaces;
}

const std::vector<ID>& TopologyEdge::getFaceIds() const
{
    return mFaces;
}

ID TopologyEdge::getOtherFaceId(ID faceId)
{
    return faceId == mFaces[0] ? mFaces[1] : mFaces[0];
}

TopologyFeature::Type TopologyEdge::getType() const
{
    return Type::EDGE;
}
