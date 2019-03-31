#ifndef BVLEAFDATA_H
#define BVLEAFDATA_H

#include "BVData.h"

#include <memory>


class CollisionObject;


class BVLeafData : public BVData
{
public:
    BVLeafData(
            Node<BVChildrenData*, BVLeafData*>* node,
            std::shared_ptr<BoundingVolume> boundingVolume,
            std::shared_ptr<CollisionObject> collisionObject);

    CollisionObject* getCollisionObject();

    // BVData interface
public:
    virtual void update();

private:
    std::shared_ptr<CollisionObject> mCollisionObject;

};

#endif // BVLEAFDATA_H
