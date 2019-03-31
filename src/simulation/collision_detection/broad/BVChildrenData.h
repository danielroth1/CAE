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
            const std::shared_ptr<BoundingVolume>& boundingVolume,
            const std::shared_ptr<BoundingVolume>& child1,
            const std::shared_ptr<BoundingVolume>& child2);

    BoundingVolume* getChild1();
    BoundingVolume* getChild2();

    // BVData interface
public:
    virtual void update();


private:
    std::shared_ptr<BoundingVolume> mChild1;
    std::shared_ptr<BoundingVolume> mChild2;

};

#endif // BVCHILDRENDATA_H
