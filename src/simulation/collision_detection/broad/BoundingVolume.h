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

    // Leaf node update method
    virtual void update(CollisionObject& collisionObject) = 0;

    // Children node update method
    virtual void update(BoundingVolume* bv1, BoundingVolume* bv2) = 0;

    virtual Type getType() const = 0;

    virtual Eigen::Vector getPosition() const = 0;
};

#endif // BOUNDINGVOLUME_H
