#include "TopologyVertex.h"


TopologyVertex::TopologyVertex(ID id)
    : TopologyFeature(id)
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

std::vector<ID>& TopologyVertex::getCellIds()
{
    return mCells;
}

const std::vector<ID>& TopologyVertex::getCellIds() const
{
    return mCells;
}

TopologyFeature::Type TopologyVertex::getType() const
{
    return Type::VERTEX;
}
