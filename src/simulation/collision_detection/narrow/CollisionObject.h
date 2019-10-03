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

    virtual void accept(CollisionObjectVisitor& visitor) = 0;

    virtual Type getType() const = 0;

    virtual Eigen::Vector getPosition() = 0;
};

#endif // COLLISIONOBJECT_H
