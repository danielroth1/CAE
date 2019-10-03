#include "BVAABB.h"
#include "BoundingVolumeVisitor.h"

#include <simulation/collision_detection/narrow/CollisionObject.h>
#include <simulation/collision_detection/narrow/CollisionSphere.h>

BVAABB::BVAABB()
{

}

void BVAABB::accept(BoundingVolumeVisitor& visitor)
{
    visitor.visit(this);
}

bool BVAABB::intersects(BoundingVolume* bv)
{
    return mBB.intersects(static_cast<BVAABB*>(bv)->getBoundingBox());
}

void BVAABB::update(CollisionObject& collisionObject)
{
    switch(collisionObject.getType())
    {
    case CollisionObject::Type::SPHERE:
    {
        CollisionSphere* cs = static_cast<CollisionSphere*>(&collisionObject);
        mBB.min() = cs->getPosition() - cs->getRadius() * Eigen::Vector::Ones();
        mBB.mid() = cs->getPosition();
        mBB.max() = cs->getPosition() + cs->getRadius() * Eigen::Vector::Ones();
        break;
    }
    case CollisionObject::Type::TRIANGLE:
    {
        break;
    }
    }
}

void BVAABB::update(BoundingVolume* bv1,BoundingVolume* bv2)
{
    BVAABB* aabb1 = static_cast<BVAABB*>(bv1);
    BVAABB* aabb2 = static_cast<BVAABB*>(bv2);

    mBB.min() = aabb1->getBoundingBox().min().cwiseMin(
                aabb2->getBoundingBox().min());

    mBB.max() = aabb1->getBoundingBox().max().cwiseMax(
                aabb2->getBoundingBox().max());

    mBB.mid() = 0.5 * (mBB.min() + mBB.max());

}

BoundingVolume::Type BVAABB::getType() const
{
    return BoundingVolume::Type::AABB;
}

Vector BVAABB::getPosition() const
{
//    return mBB.min();
    return mBB.mid();
}

const BoundingBox& BVAABB::getBoundingBox() const
{
    return mBB;
}
