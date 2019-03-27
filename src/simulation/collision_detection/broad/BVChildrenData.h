#ifndef BVCHILDRENDATA_H
#define BVCHILDRENDATA_H

#include "BVData.h"

#include <vector>

class BoundingVolume;

class BVChildrenData : public BVData
{
public:
    BVChildrenData(
            Node<BVChildrenData*, BVLeafData*>* node,
            BoundingVolume* boundingVolume,
            BoundingVolume* child1,
            BoundingVolume* child2);

    BoundingVolume* getChild1();
    BoundingVolume* getChild2();

    // BVData interface
public:
    virtual void update();


private:
    BoundingVolume* mChild1;
    BoundingVolume* mChild2;

};

#endif // BVCHILDRENDATA_H
