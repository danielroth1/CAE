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

const ID& TopologyFeature::getIDRef() const
{
    return mId;
}

void TopologyFeature::setID(ID id)
{
    mId = id;
}
