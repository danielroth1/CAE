#include "TopologyCell.h"

TopologyCell::TopologyCell(ID id)
    : TopologyFeature(id)
{

}


std::array<unsigned int, 4>& TopologyCell::getVertexIds()
{
    return mVertexIds;
}

const std::array<unsigned int, 4>& TopologyCell::getVertexIds() const
{
    return mVertexIds;
}

std::array<unsigned int, 6>& TopologyCell::getEdgeIds()
{
    return mEdgeIds;
}

const std::array<unsigned int, 6>& TopologyCell::getEdgeIds() const
{
    return mEdgeIds;
}

std::vector<unsigned int>& TopologyCell::getFaceIds()
{
    return mFaceIds;
}

const std::vector<unsigned int>& TopologyCell::getFaceIds() const
{
    return mFaceIds;
}

TopologyFeature::Type TopologyCell::getType() const
{
    return TopologyFeature::Type::CELL;
}
