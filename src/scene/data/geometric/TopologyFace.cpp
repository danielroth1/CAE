#include "TopologyFace.h"

TopologyFace::TopologyFace(ID id)
    : TopologyFeature(id)
{

}

std::array<unsigned int, 3>& TopologyFace::getVertexIds()
{
    return mVertices;
}

const std::array<unsigned int, 3>& TopologyFace::getVertexIds() const
{
    return mVertices;
}

std::array<unsigned int, 3>& TopologyFace::getEdgeIds()
{
    return mEdges;
}

const std::array<unsigned int, 3>& TopologyFace::getEdgeIds() const
{
    return mEdges;
}

std::vector<ID>& TopologyFace::getCellIds()
{
    return mCells;
}

const std::vector<ID>& TopologyFace::getCellIds() const
{
    return mCells;
}

std::vector<ID>& TopologyFace::getAdjacentFaces()
{
    return mAdjacentFaces;
}

const std::vector<ID>& TopologyFace::getAdjacentFaces() const
{
    return mAdjacentFaces;
}

TopologyFeature::Type TopologyFace::getType() const
{
    return Type::FACE;
}
