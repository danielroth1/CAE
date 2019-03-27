#include "Collider.h"
#include "CollisionObject.h"
#include "CollisionSphere.h"

#include <iostream>
#include <new>

Collider::Collider()
    : mCollisionObjectDispatcher(*this)
    , mCollisionSphereDispatcher(*this)
    , mCollisionTriangleDispatcher(*this)
{

}

bool Collider::collides(CollisionObject& co1, CollisionObject& co2)
{
    Collision c;
    bool isColliding = collides(co1, co2, c);
    if (isColliding)
    {
        mCollisions.push_back(c);
        return true;
    }
    return false;
}

void Collider::clear()
{
    mCollisions.clear();
}

std::vector<Collision>& Collider::getCollisions()
{
    return mCollisions;
}

bool Collider::collides(
        CollisionObject& co1,
        CollisionObject& co2,
        Collision& collisionReturnValue)
{
    mCollisionObjectDispatcher.collisionObject = &co1;
    mCollisionObjectDispatcher.collision = &collisionReturnValue;
    co2.accept(mCollisionObjectDispatcher);

    return mCollisionObjectDispatcher.returnValue;
}

bool Collider::collides(
        CollisionObject& co,
        CollisionSphere& cs,
        Collision& collisionReturnValue)
{
    mCollisionSphereDispatcher.collisionSphere = &cs;
    mCollisionSphereDispatcher.collision = &collisionReturnValue;
    co.accept(mCollisionSphereDispatcher);

    return mCollisionSphereDispatcher.returnValue;
}

bool Collider::collides(
        CollisionObject& co,
        CollisionTriangle& ct,
        Collision& collisionReturnValue)
{
    mCollisionTriangleDispatcher.collisionTriangle = &ct;
    mCollisionTriangleDispatcher.collision = &collisionReturnValue;
    co.accept(mCollisionTriangleDispatcher);

    return mCollisionTriangleDispatcher.returnValue;
}

bool Collider::collides(
        CollisionSphere& cs1,
        CollisionSphere& cs2,
        Collision& collisionReturnValue)
{
    bool returnValue = (cs1.getPosition() - cs2.getPosition()).norm() <
            (cs1.getRadius() + cs2.getRadius());

    if (!returnValue)
        return returnValue; // There is no collision

//    if (returnValue)
//        std::cout << "pos1 = " << cs1.getPosition().transpose() <<
//                     ", pos2 = " << cs1.getPosition().transpose() <<
//                     ", radius1 = " << cs1.getRadius() <<
//                     ", radius2 = " << cs2.getRadius() << "\n";

    // Contact normal is always directed from the second body to the first one.
    Eigen::Vector normal = (cs1.getPosition() - cs2.getPosition()).normalized();
    Eigen::Vector pointA = cs1.getPosition() - cs1.getRadius() * normal;
    Eigen::Vector pointB = cs2.getPosition() + cs2.getRadius() * normal;

    double depth = (pointA - pointB).norm();

    // Calculate normal
    new (&collisionReturnValue) Collision(cs1.getPointRef().getSimulationObject(),
                                          cs2.getPointRef().getSimulationObject(),
                                          pointA, pointB, normal, depth,
                                          cs1.getVertexIndex(),
                                          cs2.getVertexIndex());

    return returnValue;
}

bool Collider::collides(
        CollisionSphere& /*cs*/,
        CollisionTriangle& /*ct*/,
        Collision& /*collisionReturnValue*/)
{
    // TODO: implement this
    return false;
}

bool Collider::collides(
        CollisionTriangle& /*ct1*/,
        CollisionTriangle& /*ct2*/,
        Collision& /*collisionReturnValue*/)
{
    // TODO: implement this
    return false;
}

// CollisionObjectCollisionObjectDispatcher

void Collider::CollisionObjectDispatcher::visit(CollisionSphere* collisionSphere)
{
    returnValue = collider.collides(*collisionObject, *collisionSphere, *collision);
}

void Collider::CollisionObjectDispatcher::visit(CollisionTriangle* collisionTriangle)
{
    returnValue = collider.collides(*collisionObject, *collisionTriangle, *collision);
}

// CollisionObjectCollisionSphereDispatcher

void Collider::CollisionSphereDispatcher::visit(CollisionSphere* cs)
{
    returnValue = collider.collides(*collisionSphere, *cs, *collision);
}

void Collider::CollisionSphereDispatcher::visit(CollisionTriangle* collisionTriangle)
{
    returnValue = collider.collides(*collisionSphere, *collisionTriangle, *collision);
}

// CollisionObjectCollisionTriangleDispatcher

void Collider::CollisionTriangleDispatcher::visit(CollisionSphere* cs)
{
    returnValue = collider.collides(*cs, *collisionTriangle, *collision);
}

void Collider::CollisionTriangleDispatcher::visit(CollisionTriangle* ct)
{
    returnValue = collider.collides(*collisionTriangle, *ct, *collision);
}
