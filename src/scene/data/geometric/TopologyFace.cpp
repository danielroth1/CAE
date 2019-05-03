#include "TopologyFace.h"

TopologyFace::TopologyFace()
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

TopologyFeature::Type TopologyFace::getType() const
{
    return Type::FACE;
}
