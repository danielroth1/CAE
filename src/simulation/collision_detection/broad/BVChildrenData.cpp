#include "BVChildrenData.h"
#include "BoundingVolume.h"

BVChildrenData::BVChildrenData(
        Node<BVChildrenData*, BVLeafData*>* node,
        const std::shared_ptr<BoundingVolume>& boundingVolume,
        const std::shared_ptr<BoundingVolume>& child1,
        const std::shared_ptr<BoundingVolume>& child2)
    : BVData(node, boundingVolume)
    , mChild1(child1)
    , mChild2(child2)
{

}

BoundingVolume* BVChildrenData::getChild1()
{
    return mChild1.get();
}

BoundingVolume* BVChildrenData::getChild2()
{
    return mChild2.get();
}

void BVChildrenData::update()
{
    mBoundingVolume->update(mChild1.get(), mChild2.get());
}
