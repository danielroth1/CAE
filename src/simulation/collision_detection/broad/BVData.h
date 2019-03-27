#ifndef BVDATA_H
#define BVDATA_H

//#include "BVChildrenData.h"
//#include "BVLeafData.h"

#include <data_structures/tree/NodeData.h>

class BVChildrenData;
class BVLeafData;

class BoundingVolume;

class BVData// : public NodeData<BVChildrenData*, BVLeafData*>
{
public:
    BVData(Node<BVChildrenData*, BVLeafData*>* node, BoundingVolume* boundingVolume);

    BoundingVolume* getBoundingVolume();

    virtual void update() = 0;

protected:
    virtual ~BVData();

    BoundingVolume* mBoundingVolume;
};

#endif // BVDATA_H
