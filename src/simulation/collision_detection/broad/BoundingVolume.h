#ifndef BOUNDINGVOLUME_H
#define BOUNDINGVOLUME_H

#include <data_structures/DataStructures.h>
#include <data_structures/tree/NodeData.h>

class BoundingVolumeVisitor;
class CollisionObject;


class BoundingVolume
{
public:
    enum class Type
    {
        SPHERE, AABB
    };

    BoundingVolume();
    virtual ~BoundingVolume();

    virtual void accept(BoundingVolumeVisitor& visitor) = 0;

    virtual bool intersects(BoundingVolume* bv) = 0;

    // Checks if the given point is inside the bounding volume.
    // \param margin - boundng volume is extended by this margin
    virtual bool isInside(const Eigen::Vector3d& point, double margin) = 0;

    // Leaf node update method
    virtual void update(CollisionObject& collisionObject,
                        double collisionMargin) = 0;

    // Children node update method
    virtual void update(BoundingVolume* bv1, BoundingVolume* bv2) = 0;

    virtual Type getType() const = 0;

    virtual Eigen::Vector getPosition() const = 0;

    // Returns a size value that is used when deciding which node to iterate
    // next to in the BVH collision check.
    virtual double getSize() const = 0;
};

#endif // BOUNDINGVOLUME_H
