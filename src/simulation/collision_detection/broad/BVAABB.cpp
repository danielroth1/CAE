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
        mBB.mid() = cs->getPosition();
        mBB.min() = mBB.mid() - cs->getRadius() * Eigen::Vector::Ones();
        mBB.max() = mBB.mid() + cs->getRadius() * Eigen::Vector::Ones();
        mBB.size() = mBB.max() - mBB.min();
        break;
    }
    case CollisionObject::Type::TRIANGLE:
    {
        break;
    }
    }
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
