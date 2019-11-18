#ifndef COLLIDER_H
#define COLLIDER_H

#include "Collision.h"
#include "CollisionObjectVisitor.h"
#include <vector>

class CollisionObject;
class CollisionSphere;
class CollisionTriangle;

class Collider
{
public:
    // Constructor
        Collider();

        // Saves collisions in mCollisions of this class.
        bool collides(
                CollisionObject& co1,
                CollisionObject& co2);

        // Removes currently stored collisions.
        void clear();

        std::vector<Collision>& getCollisions();

        bool getInvertNormalsIfNecessary() const;
        void setInvertNormalsIfNecessary(bool invertNormalsIfNecessary);

    // Mthods for dispatching
        bool collides(
                CollisionObject& co1,
                CollisionObject& co2,
                Collision& collisionReturnValue);

private:

    // Mthods for dispatching
        bool collides(
                CollisionObject& co,
                CollisionSphere& cs,
                Collision& collisionReturnValue);

        bool collides(
                CollisionObject& co,
                CollisionTriangle& ct,
                Collision& collisionReturnValue);

    // Collision Methods
        bool collides(
                CollisionSphere& cs1,
                CollisionSphere& cs2,
                Collision& collisionReturnValue);

        bool collides(
                CollisionSphere& cs,
                CollisionTriangle& ct,
                Collision& collisionReturnValue);

        bool collides(
                CollisionTriangle& ct1,
                CollisionTriangle& ct2,
                Collision& collisionReturnValue);

        // \param ct1 - source
        // \param ct2 - target
        bool collidesTriangle(
                CollisionTriangle& ct1,
                CollisionTriangle& ct2,
                Collision& collisionReturnValue);

        bool collidesTrianglesPair(
                CollisionTriangle& ct1,
                CollisionTriangle& ct2,
                double marginSquared,
                Collision& collisionReturnValue);

        bool collidesEdgesPair(
                CollisionTriangle& ct1,
                CollisionTriangle& ct2,
                double marginSquared,
                Collision& collisionReturnValue);

    bool isInside(CollisionSphere& cs1, CollisionSphere& cs2);

    bool passesFaceNormalTest(CollisionSphere& cs1, Eigen::Vector normal);

    std::vector<Collision> mCollisions;
    bool mInvertNormalsIfNecessary;

};

#endif // COLLIDER_H
