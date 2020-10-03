#ifndef BOUNDINGVOLUMEFACTORY_H
#define BOUNDINGVOLUMEFACTORY_H


#include "BoundingVolume.h"

#include <memory>
#include <vector>

class BVAABB;
class BVSphere;
class CollisionObject;
class AbstractPolygon;
class SimulationObject;

class BoundingVolumeFactory
{
public:
    BoundingVolumeFactory();

    static std::shared_ptr<BoundingVolume> createBoundingVolume(
            CollisionObject& co,
            AbstractPolygon& polygon,
            BoundingVolume::Type bvType);

    static std::shared_ptr<BoundingVolume> createBoundingVolume(
            BoundingVolume* bv1,
            BoundingVolume* bv2,
            AbstractPolygon& polygon,
            BoundingVolume::Type bvType);

    static std::shared_ptr<BVSphere> createBVSphere(CollisionObject& co, AbstractPolygon& polygon);

    // Creates a BVSphere that surrounds the given BVSpheres.
    static std::shared_ptr<BVSphere> createBVSphere(BVSphere* sphere1, BVSphere* sphere2, AbstractPolygon& polygon);

    static std::shared_ptr<BVAABB> createBVAABB(CollisionObject& co, AbstractPolygon& polygon);

    // Creates a BVAABB that surrounds the given BVAABBs.
    static std::shared_ptr<BVAABB> createBVAABB(BVAABB* bv1, BVAABB* bv2, AbstractPolygon& polygon);
};

#endif // BOUNDINGVOLUMEFACTORY_H
