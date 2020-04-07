#include "TopologyFeature.h"

TopologyFeature::TopologyFeature(ID id, ID geometryId)
    : mId(id)
    , mGeometryID(geometryId)
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
