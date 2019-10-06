#ifndef COLLISIONOBJECT_H
#define COLLISIONOBJECT_H

#include <data_structures/DataStructures.h>

class CollisionObjectVisitor;


class CollisionObject
{
public:
    enum class Type
    {
        SPHERE, TRIANGLE
    };

    CollisionObject();
    virtual ~CollisionObject();

    // Updates the world space position coordinates of the reference point,
    // e.g. applying the rotation matrix if in BODY_SPACE representation.
    // Call this method when the position changed and before calling getPosition().
    virtual void update() = 0;

    // Same as update() but with the previous position.
    virtual void updatePrevious() = 0;

    virtual void accept(CollisionObjectVisitor& visitor) = 0;

    virtual Type getType() const = 0;

    virtual Eigen::Vector getPosition() = 0;
};

#endif // COLLISIONOBJECT_H
