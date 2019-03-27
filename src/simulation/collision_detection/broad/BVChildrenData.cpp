#include "BVChildrenData.h"
#include "BoundingVolume.h"

BVChildrenData::BVChildrenData(
        Node<BVChildrenData*, BVLeafData*>* node,
        BoundingVolume* boundingVolume,
        BoundingVolume* child1,
        BoundingVolume* child2)
    : BVData(node, boundingVolume)
    , mChild1(child1)
    , mChild2(child2)
{

}

BoundingVolume*BVChildrenData::getChild1()
{
    return mChild1;
}

BoundingVolume*BVChildrenData::getChild2()
{
    return mChild2;
}

void BVChildrenData::update()
{
    mBoundingVolume->update(mChild1, mChild2);
}
