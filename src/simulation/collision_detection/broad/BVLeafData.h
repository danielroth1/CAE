#ifndef BVLEAFDATA_H
#define BVLEAFDATA_H

#include "BVData.h"


class CollisionObject;


class BVLeafData : public BVData
{
public:
    BVLeafData(
            Node<BVChildrenData*, BVLeafData*>* node,
            BoundingVolume* boundingVolume,
            CollisionObject* collisionObject);

    CollisionObject* getCollisionObject();

    // BVData interface
public:
    virtual void update();

private:
    CollisionObject* mCollisionObject;

};

#endif // BVLEAFDATA_H
