#ifndef COLLIDER_H
#define COLLIDER_H

#include "Collision.h"
#include "CollisionObjectVisitor.h"

#include <boost/functional/hash.hpp>
#include <unordered_set>
#include <vector>

class CollisionObject;
class CollisionSphere;
class CollisionTriangle;
class MeshInterpolatorFEM;
class TopologyEdge;
class TopologyFace;
class TopologyVertex;
class TriangleCollider;

class Collider
{
public:

    typedef std::tuple<SimulationObject*, ID, SimulationObject*, ID> CollisionTuple;
    typedef std::unordered_set<CollisionTuple, boost::hash<CollisionTuple>> CollisionTupleSet;

    // Constructor
        Collider(double collisionMargin);

        // Saves collisions in mCollisions of this class.
        bool collides(
                CollisionObject& co1,
                CollisionObject& co2,
                const CollisionTupleSet& ignoreFeatures);

        bool collidesTrianglesImproved(
                CollisionObject& co1,
                CollisionObject& co2,
                const CollisionTupleSet& ignoreFeatures);

        // Removes currently stored collisions.
        void clear();

        void prepare(
                AbstractPolygon* poly1, AbstractPolygon* poly2,
                SimulationObject* so1, SimulationObject* so2,
                MeshInterpolatorFEM* interpolator1, MeshInterpolatorFEM* interpolator2,
                int runId);

        // Performs the narrow phase collision detection for the feature pairs.
        // Call this method after calling all collider for close CollisionObjects
        // but before getCollisions().
        bool evaluate();

        // Returns the collisions that were found in the collides method call.
        std::vector<Collision>& getCollisions();

        bool getInvertNormalsIfNecessary() const;
        void setInvertNormalsIfNecessary(bool invertNormalsIfNecessary);

        void setCollisionMargin(double collisionMargin);
        double getCollisionMargin() const;

        // Deprecated: use collider(CollisionObject&, CollisionObject&) instead
        // Perfoms a collision detetion between the given collision objects.
        // A a lot faster method would be to use the TriangleCollider which
        // is done in the other collide method.
        bool collides(
                CollisionObject& co1,
                CollisionObject& co2,
                Collision& collisionReturnValue);

        // Checks if the distance between the vertex and the face is within the
        // collision margin and if so, creates a collision and stores it in
        // the given collision parameter.
        bool collides(
                TopologyVertex& v,
                TopologyFace& f,
                SimulationObject* so1,
                SimulationObject* so2,
                MeshInterpolatorFEM* interpolator1,
                MeshInterpolatorFEM* interpolator2,
                AbstractPolygon* poly1,
                AbstractPolygon* poly2,
                bool revertedFeaturePair,
                Collision& collision);

        // Checks if the distance between the two edges is within the
        // collision margin and if so, creates a collision and stores it in
        // the given collision parameter.
        bool collides(TopologyEdge& e1,
                     TopologyEdge& e2,
                     SimulationObject* so1,
                     SimulationObject* so2,
                     MeshInterpolatorFEM* interpolator1,
                     MeshInterpolatorFEM* interpolator2,
                     AbstractPolygon* poly1,
                     AbstractPolygon* poly2,
                     Collision& collision);

private:

    // Methods for dispatching
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
