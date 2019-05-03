#include "TopologyFeature.h"

TopologyFeature::TopologyFeature(ID id)
    : mId(id)
{

}

TopologyFeature::~TopologyFeature()
{

}

ID TopologyFeature::getID() const
{
    return mId;
}
