#include "TopologyEdge.h"

#include <iostream>

TopologyEdge::TopologyEdge(ID id, ID geometryId)
    : TopologyFeature(id, geometryId)
{
    mFaces.reserve(2);
}

TopologyFeature::Type TopologyEdge::getType() const
{
    return Type::EDGE;
}
