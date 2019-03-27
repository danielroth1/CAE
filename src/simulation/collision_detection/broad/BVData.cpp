#include "BVData.h"


BVData::BVData(Node<BVChildrenData*, BVLeafData*>* node, BoundingVolume* boundingVolume)
    //: NodeData<BVChildrenData*, BVLeafData*>(node)
    : mBoundingVolume(boundingVolume)
{

}

BoundingVolume* BVData::getBoundingVolume()
{
    return mBoundingVolume;
}

BVData::~BVData()
{

}
