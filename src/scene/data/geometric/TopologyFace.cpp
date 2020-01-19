#include "TopologyFace.h"

TopologyFace::TopologyFace(ID id)
    : TopologyFeature(id)
{
    mFeatureOwnership.reset(); // Sets bits to zero.
}
