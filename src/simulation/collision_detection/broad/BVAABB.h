#ifndef BVAABB_H
#define BVAABB_H

#include "BoundingVolume.h"
#include "BoundingVolumeVisitor.h"

#include <data_structures/BoundingBox.h>

#include <simulation/collision_detection/narrow/CollisionObject.h>
#include <simulation/collision_detection/narrow/CollisionSphere.h>

// Bounding Volume Axis Aligned Bounding Box
class BVAABB : public BoundingVolume
{
public:
    BVAABB();

    // BoundingVolume interface
public:
    virtual void accept(BoundingVolumeVisitor& visitor) override final
    {
        visitor.visit(this);
    }

    virtual bool intersects(BoundingVolume* bv) override final
    {
        return mBB.intersects(static_cast<BVAABB*>(bv)->getBoundingBox());
    }

    virtual void update(CollisionObject& collisionObject) override final
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

    virtual void update(BoundingVolume* bv1, BoundingVolume* bv2) override final
    {
        mBB.min() = static_cast<BVAABB*>(bv1)->getBoundingBox().min().cwiseMin(
                    static_cast<BVAABB*>(bv2)->getBoundingBox().min());

        mBB.max() = static_cast<BVAABB*>(bv1)->getBoundingBox().max().cwiseMax(
                    static_cast<BVAABB*>(bv2)->getBoundingBox().max());

        mBB.size() = mBB.max() - mBB.min();

        //    mBB.mid() = 0.5 * (mBB.min() + mBB.max());

    }

    virtual Type getType() const final
    {
        return BoundingVolume::Type::AABB;
    }

    // Returns the bounding boxes minimum position.
    virtual Eigen::Vector getPosition() const final
    {
        //    return mBB.min();
        return mBB.mid();
    }

    virtual double getSize() const override final
    {
        return mBB.size()(0);
    }

    const BoundingBox& getBoundingBox() const
    {
        return mBB;
    }

private:
    BoundingBox mBB;

};

#endif // BVAABB_H
