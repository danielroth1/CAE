#ifndef COLLIDER_H
#define COLLIDER_H

#include "Collision.h"
#include "CollisionObjectVisitor.h"
#include <vector>

class CollisionObject;
class CollisionSphere;
class CollisionTriangle;
class MeshInterpolatorFEM;
class TriangleCollider;

class Collider
{
public:
    // Constructor
        Collider(double collisionMargin);

        // Saves collisions in mCollisions of this class.
        bool collides(
                CollisionObject& co1,
                CollisionObject& co2);

        bool collidesTrianglesImproved(
                CollisionObject& co1,
                CollisionObject& co2);

        // Removes currently stored collisions.
        void clear();

        void prepare(
                Polygon* poly1, Polygon* poly2,
                SimulationObject* so1, SimulationObject* so2,
                MeshInterpolatorFEM* interpolator1, MeshInterpolatorFEM* interpolator2,
                int runId);

        // Performs the narrow phase collision detection for the feature pairs.
        // Call this method after calling all collider for close CollisionObjects
        // but before getCollisions().
        bool evaluate();

        std::vector<Collision>& getCollisions();

        bool getInvertNormalsIfNecessary() const;
        void setInvertNormalsIfNecessary(bool invertNormalsIfNecessary);

        void setCollisionMargin(double collisionMargin);
        double getCollisionMargin() const;

    // Mthods for dispatching
        bool collides(
                CollisionObject& co1,
                CollisionObject& co2,
                Collision& collisionReturnValue);

private:

    // Mthods for dispatching
        // Deprecated: use TriangleCollider instead
        bool collides(
                CollisionObject& co,
                CollisionSphere& cs,
                Collision& collisionReturnValue);

        // Deprecated: use TriangleCollider instead
        bool collides(
                CollisionObject& co,
                CollisionTriangle& ct,
                Collision& collisionReturnValue);

    // Collision Methods
        bool collides(
                CollisionSphere& cs1,
                CollisionSphere& cs2,
                Collision& collisionReturnValue);

        // Deprecated: use TriangleCollider instead
        bool collides(
                CollisionSphere& cs,
                CollisionTriangle& ct,
                Collision& collisionReturnValue);

        // Deprecated: use TriangleCollider instead
        bool collides(
                CollisionTriangle& ct1,
                CollisionTriangle& ct2,
                Collision& collisionReturnValue);

        // Deprecated: use TriangleCollider instead
        // \param ct1 - source
        // \param ct2 - target
        bool collidesTriangle(
                CollisionTriangle& ct1,
                CollisionTriangle& ct2,
                Collision& collisionReturnValue);

        // Deprecated: use TriangleCollider instead
        bool collidesTrianglesPair(
                CollisionTriangle& ct1,
                CollisionTriangle& ct2,
                double marginSquared,
                Collision& collisionReturnValue);

        // Deprecated: use TriangleCollider instead
        bool collidesEdgesPair(
                CollisionTriangle& ct1,
                CollisionTriangle& ct2,
                double marginSquared,
                Collision& collisionReturnValue);

    bool isInside(CollisionSphere& cs1, CollisionSphere& cs2);

    bool passesFaceNormalTest(CollisionSphere& cs1, Eigen::Vector normal);

    std::vector<Collision> mCollisions;
    bool mInvertNormalsIfNecessary;

    std::shared_ptr<TriangleCollider> mTriangleCollider;

    double mCollisionMargin;
    double mCollisionMarginSquared;
};

#endif // COLLIDER_H
