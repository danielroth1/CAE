#include "BVLeafData.h"
#include "BoundingVolume.h"


BVLeafData::BVLeafData(
        Node<BVChildrenData*, BVLeafData*>* node,
        std::shared_ptr<BoundingVolume> boundingVolume,
        std::shared_ptr<CollisionObject> collisionObject)
    : BVData(node, boundingVolume)
    , mCollisionObject(collisionObject)
{

}

CollisionObject* BVLeafData::getCollisionObject()
{
    return mCollisionObject.get();
}

void BVLeafData::update()
{
    mBoundingVolume->update(*mCollisionObject);
}
