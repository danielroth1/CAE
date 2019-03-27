#ifndef BOUNDINGVOLUMEFACTORY_H
#define BOUNDINGVOLUMEFACTORY_H


#include <vector>

class BoundingVolume;
class BVSphere;
class CollisionObject;
class Polygon;
class SimulationObject;

class BoundingVolumeFactory
{
public:
    BoundingVolumeFactory();

    static BVSphere* createBVSphere(CollisionObject& co, Polygon& polygon);

    // Creates a BVSphere that surrounds the given BVSpheres.
    static BVSphere* createBVSphere(BVSphere* sphere1, BVSphere* sphere2, Polygon& polygon);
};

#endif // BOUNDINGVOLUMEFACTORY_H
