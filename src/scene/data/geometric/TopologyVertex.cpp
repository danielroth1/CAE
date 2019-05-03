#include "TopologyVertex.h"

TopologyVertex::TopologyVertex()
{

}

std::vector<ID>& TopologyVertex::getEdgeIds()
{
    return mEdges;
}

const std::vector<ID>& TopologyVertex::getEdgeIds() const
{
    return mEdges;
}

std::vector<ID>& TopologyVertex::getFaceIds()
{
    return mFaces;
}

const std::vector<ID>& TopologyVertex::getFaceIds() const
{
    return mFaces;
}

TopologyFeature::Type TopologyVertex::getType() const
{
    return Type::VERTEX;
}
