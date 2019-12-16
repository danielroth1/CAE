#include "TopologyEdge.h"

#include <iostream>

TopologyEdge::TopologyEdge(ID id)
    : TopologyFeature(id)
{
    mFaces.reserve(2);
}

TopologyFeature::Type TopologyEdge::getType() const
{
    return Type::EDGE;
}
