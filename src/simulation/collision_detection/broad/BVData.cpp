#include "BVData.h"


BVData::BVData(Node<BVChildrenData*, BVLeafData*>* node,
               std::shared_ptr<BoundingVolume> boundingVolume)
    //: NodeData<BVChildrenData*, BVLeafData*>(node)
    : mBoundingVolume(boundingVolume)
{

}



BVData::~BVData()
{

}
