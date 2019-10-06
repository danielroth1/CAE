#ifndef BVAABB_H
#define BVAABB_H

#include "BoundingVolume.h"

#include <data_structures/BoundingBox.h>

// Bounding Volume Axis Aligned Bounding Box
class BVAABB : public BoundingVolume
{
public:
    BVAABB();

    // BoundingVolume interface
public:
    virtual void accept(BoundingVolumeVisitor& visitor);
    virtual bool intersects(BoundingVolume* bv);
    virtual void update(CollisionObject& collisionObject);
    virtual void update(BoundingVolume* bv1, BoundingVolume* bv2)
    {
        mBB.min() = static_cast<BVAABB*>(bv1)->getBoundingBox().min().cwiseMin(
                    static_cast<BVAABB*>(bv2)->getBoundingBox().min());

        mBB.max() = static_cast<BVAABB*>(bv1)->getBoundingBox().max().cwiseMax(
                    static_cast<BVAABB*>(bv2)->getBoundingBox().max());

        //    mBB.mid() = 0.5 * (mBB.min() + mBB.max());

    }

    virtual Type getType() const;

    // Returns the bounding boxes minimum position.
    virtual Eigen::Vector getPosition() const;

    const BoundingBox& getBoundingBox() const;

private:
    BoundingBox mBB;

};

#endif // BVAABB_H
