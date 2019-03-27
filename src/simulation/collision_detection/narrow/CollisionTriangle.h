#ifndef COLLISIONTRIANGLE_H
#define COLLISIONTRIANGLE_H

#include "CollisionObject.h"



class CollisionTriangle : public CollisionObject
{
public:
    CollisionTriangle();

    // CollisionObject interface
public:
    virtual void accept(CollisionObjectVisitor& visitor);
};

#endif // COLLISIONTRIANGLE_H
