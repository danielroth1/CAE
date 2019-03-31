#ifndef BVHDEFORMABLE_H
#define BVHDEFORMABLE_H

#include "BoundingVolumeHierarchy.h"
#include "BVHCore.h"

#include <memory>

class BVSphere;
class CollisionObject;

// Surrounds each collision object with BVSpheres, then
// creates a bounding volume hierarchy where those initially
// created BVSpheres are at the leaf level. KD-tree is
// used for subspace partitioning.
class BVHDeformable : public BoundingVolumeHierarchy
{
public:
    BVHDeformable(SimulationObject* so,
                  Polygon* polygon,
                  const std::vector<std::shared_ptr<CollisionObject>>& collisionObjects);

    // BoundingVolumeHierarchy interface
public:
    virtual void initialize();
    virtual void udpate();

private:
    void initializeWithKDTree();

    BVHNode* initializeWithKDTreeRec(
            const std::vector<std::shared_ptr<BVSphere>>& spheres,
            const std::vector<std::shared_ptr<CollisionObject>>& collisionObjects,
            std::shared_ptr<BVSphere>& boundingVolumeRet);
};

#endif // BVHDEFORMABLE_H
