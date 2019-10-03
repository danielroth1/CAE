#ifndef BOUNDINGVOLUMEFACTORY_H
#define BOUNDINGVOLUMEFACTORY_H


#include "BoundingVolume.h"

#include <memory>
#include <vector>

class BVAABB;
class BVSphere;
class CollisionObject;
class Polygon;
class SimulationObject;

class BoundingVolumeFactory
{
public:
    BoundingVolumeFactory();

    static std::shared_ptr<BoundingVolume> createBoundingVolume(
            CollisionObject& co,
            Polygon& polygon,
            BoundingVolume::Type bvType);

    static std::shared_ptr<BoundingVolume> createBoundingVolume(
            BoundingVolume* bv1,
            BoundingVolume* bv2,
            Polygon& polygon,
            BoundingVolume::Type bvType);

    static std::shared_ptr<BVSphere> createBVSphere(CollisionObject& co, Polygon& polygon);

    // Creates a BVSphere that surrounds the given BVSpheres.
    static std::shared_ptr<BVSphere> createBVSphere(BVSphere* sphere1, BVSphere* sphere2, Polygon& polygon);

    static std::shared_ptr<BVAABB> createBVAABB(CollisionObject& co, Polygon& polygon);

    // Creates a BVAABB that surrounds the given BVAABBs.
    static std::shared_ptr<BVAABB> createBVAABB(BVAABB* bv1, BVAABB* bv2, Polygon& polygon);
};

#endif // BOUNDINGVOLUMEFACTORY_H
