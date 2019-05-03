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

TopologyFeature::Type TopologyEdge::getType() const
{
    return Type::EDGE;
}
