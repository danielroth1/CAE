#ifndef BVDATA_H
#define BVDATA_H

//#include "BVChildrenData.h"
//#include "BVLeafData.h"

#include <data_structures/tree/NodeData.h>

#include <memory>


class BVChildrenData;
class BVLeafData;

class BoundingVolume;

class BVData// : public NodeData<BVChildrenData*, BVLeafData*>
{
public:
    BVData(Node<BVChildrenData*, BVLeafData*>* node,
           std::shared_ptr<BoundingVolume> boundingVolume);

    BoundingVolume* getBoundingVolume();

    virtual void update() = 0;

protected:
    virtual ~BVData();

    std::shared_ptr<BoundingVolume> mBoundingVolume;
};

#endif // BVDATA_H
