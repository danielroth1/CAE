#ifndef BVAABB_H
#define BVAABB_H

#include "BoundingVolume.h"
#include "BoundingVolumeVisitor.h"

#include <data_structures/BoundingBox.h>

#include <simulation/collision_detection/narrow/CollisionObject.h>
#include <simulation/collision_detection/narrow/CollisionSphere.h>
#include <simulation/collision_detection/narrow/CollisionTriangle.h>

#include <scene/data/geometric/Polygon2DAccessor.h>

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

    virtual bool isInside(const Eigen::Vector3d& point, double margin) override final
    {
        return mBB.isInside(point, margin);
    }

    // Doesn't use the collision margin for Spheres (because its not used
    // for anythin else than triangle-triangle collision detection).
    virtual void update(CollisionObject& collisionObject,
                        double collisionMargin) override final
    {
        switch(collisionObject.getType())
        {
        case CollisionObject::Type::SPHERE:
        {
            // Note: No collision margin used here

            CollisionSphere* cs = static_cast<CollisionSphere*>(&collisionObject);
            mBB.mid() = cs->getPosition();
            mBB.min() = mBB.mid() - cs->getRadius() * Eigen::Vector::Ones();
            mBB.max() = mBB.mid() + cs->getRadius() * Eigen::Vector::Ones();
            mBB.size() = mBB.max() - mBB.min();
            break;
        }
        case CollisionObject::Type::TRIANGLE:
        {
            CollisionTriangle* t = static_cast<CollisionTriangle*>(&collisionObject);
            const Face& f = t->getFace();
            const std::shared_ptr<Polygon2DAccessor>& acc = t->getAccessor();

            // skipped here because its not needed in the current implementation
//            mBB.mid() = 0.3333 * (
//                    acc->getPosition(f[0]) +
//                    acc->getPosition(f[1]) +
//                    acc->getPosition(f[2]));

            mBB.min() = acc->getPosition(f[0]).cwiseMin(
                            acc->getPosition(f[1])).cwiseMin(
                                acc->getPosition(f[2])) - collisionMargin * Eigen::Vector::Ones();

            mBB.max() = acc->getPosition(f[0]).cwiseMax(
                            acc->getPosition(f[1])).cwiseMax(
                                acc->getPosition(f[2])) + collisionMargin * Eigen::Vector::Ones();

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
        // skipped here because its not needed in the current implementation
//        mBB.mid() = 0.5 * (mBB.min() + mBB.max());

    }

    virtual Type getType() const final
    {
        return BoundingVolume::Type::AABB;
    }

    // Returns the bounding boxes minimum position.
    virtual Eigen::Vector getPosition() const final
    {
//        return mBB.min();
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
