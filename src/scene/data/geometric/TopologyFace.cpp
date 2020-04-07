#include "TopologyFace.h"

TopologyFace::TopologyFace(ID id, ID geometryId)
    : TopologyFeature(id, geometryId)
{
    mFeatureOwnership.reset(); // Sets bits to zero.
}
