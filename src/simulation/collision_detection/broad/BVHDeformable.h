#ifndef BVHDEFORMABLE_H
#define BVHDEFORMABLE_H

#include "BoundingVolumeHierarchy.h"
#include "BVHCore.h"

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
                  const std::vector<CollisionObject*>& collisionObjects);

    // BoundingVolumeHierarchy interface
public:
    virtual void initialize();
    virtual void udpate();

private:
    void initializeWithKDTree();

    BVHNode* initializeWithKDTreeRec(
            const std::vector<BVSphere*>& spheres,
            const std::vector<CollisionObject*>& collisionObjects,
            BVSphere*& boundingVolumeRet);
};

#endif // BVHDEFORMABLE_H
