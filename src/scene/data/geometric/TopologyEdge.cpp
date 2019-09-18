#include "TopologyEdge.h"

#include <iostream>

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

std::vector<ID>& TopologyEdge::getCellIds()
{
    return mCells;
}

const std::vector<ID>& TopologyEdge::getCellIds() const
{
    return mCells;
}

ID TopologyEdge::getOtherFaceId(ID faceId)
{
//    if (mFaces.size() != 2)
//    {
//        std::cout << "Error: edge doesn't have exactly 2 neighbored faces but "
//                     "instead " << mFaces.size() << ".\n";
//    }
    return faceId == mFaces[0] ? mFaces[1] : mFaces[0];
}

TopologyFeature::Type TopologyEdge::getType() const
{
    return Type::EDGE;
}
