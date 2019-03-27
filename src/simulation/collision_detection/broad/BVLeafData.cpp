#include "BVLeafData.h"
#include "BoundingVolume.h"


BVLeafData::BVLeafData(
        Node<BVChildrenData*, BVLeafData*>* node,
        BoundingVolume* boundingVolume,
        CollisionObject* collisionObject)
    : BVData(node, boundingVolume)
    , mCollisionObject(collisionObject)
{

}

CollisionObject* BVLeafData::getCollisionObject()
{
    return mCollisionObject;
}

void BVLeafData::update()
{
    mBoundingVolume->update(*mCollisionObject);
}
