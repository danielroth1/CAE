#ifndef COLLISIONOBJECT_H
#define COLLISIONOBJECT_H

#include <data_structures/DataStructures.h>

class CollisionObjectVisitor;


class CollisionObject
{
public:
    CollisionObject();
    virtual ~CollisionObject();

    virtual void accept(CollisionObjectVisitor& visitor) = 0;

    virtual Eigen::Vector getPosition() = 0;
};

#endif // COLLISIONOBJECT_H
