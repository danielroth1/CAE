#include "Collider.h"
#include "CollisionObject.h"
#include "CollisionSphere.h"

#include <iostream>
#include <new>

#include <scene/data/geometric/Polygon.h>
#include <scene/data/geometric/Polygon2D.h>
#include <scene/data/geometric/Polygon2DData.h>
#include <scene/data/geometric/Polygon3D.h>
#include <scene/data/geometric/PolygonTopology.h>
#include <scene/data/geometric/TopologyFeature.h>

#include <simulation/SimulationObject.h>

#include <scene/data/GeometricData.h>

Collider::Collider()
    : mInvertNormalsIfNecessary(false)
    , mCollisionObjectDispatcher(*this)
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

bool Collider::getInvertNormalsIfNecessary() const
{
    return mInvertNormalsIfNecessary;
}

void Collider::setInvertNormalsIfNecessary(bool invertNormalsIfNecessary)
{
    mInvertNormalsIfNecessary = invertNormalsIfNecessary;
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

    // calculate previous points
//    Eigen::Vector normalPrev =
//            (cs1.getPointRef().getPointPrevious() -
//             cs2.getPointRef().getPointPrevious()).normalized();

//    Eigen::Vector pointAPrev =
//            cs1.getPointRef().getPointPrevious() -
//            cs1.getRadius() * normalPrev;

//    Eigen::Vector pointBPrev =
//            cs2.getPointRef().getPointPrevious() +
//            cs2.getRadius() * normalPrev;

    double depth = (pointA - pointB).norm();
    bool isIn = false;

    if (mInvertNormalsIfNecessary)
    {
        bool isIn1 = isInside(cs1, cs2);

        bool passesFaceNormalCheck1 = true;
        if (isIn1)
        {
            passesFaceNormalCheck1 = passesFaceNormalTest(cs1, -normal);
        }

        bool isIn2 = isInside(cs2, cs1);

        bool passesFaceNormalCheck2 = true;
        if (isIn2)
        {
            passesFaceNormalCheck2 = passesFaceNormalTest(cs2, normal);
        }

        isIn = isIn1 && isIn2;
//        isIn = isIn1 || isIn2;
        // if is inside, revert the normal
        if (isIn)
        {
            normal *= -1;

            if (!passesFaceNormalCheck1 || !passesFaceNormalCheck2)
            {
                return false;
            }

    //        return false;
        }
    }

    // Calculate normal
    new (&collisionReturnValue) Collision(cs1.getPointRef().getSimulationObject(),
                                          cs2.getPointRef().getSimulationObject(),
                                          pointA, pointB, normal, depth,
                                          cs1.getVertexIndex(),
                                          cs2.getVertexIndex(),
                                          isIn);

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

bool Collider::isInside(CollisionSphere& cs1, CollisionSphere& cs2)
{
    bool isInside = true;
    if (cs1.getTopologyFeature() != nullptr)
    {
        GeometricData* g1 = cs1.getPointRef().getSimulationObject()->getGeometricData();
        if (g1->getType() == GeometricData::Type::POLYGON)
        {
            Polygon* p1 = static_cast<Polygon*>(g1);
//            isInside = p1->isInside(
//                        *cs1.getTopologyFeature().get(),
//                        cs1.getPosition(),
//                        cs1.getRadius() * 100,
//                        cs2.getPosition());
            isInside =
                    p1->isInside(
                        *cs1.getTopologyFeature().get(),
                        cs2.getPosition());
        }
    }
    return isInside;
}

bool Collider::passesFaceNormalTest(CollisionSphere& cs1, Eigen::Vector normal)
{
    double toleranceAngleDegree = 15.0;
    double toleranceRad = toleranceAngleDegree / 180.0 * M_PI;

    if (cs1.getTopologyFeature() != nullptr)
    {
        GeometricData* g1 = cs1.getPointRef().getSimulationObject()->getGeometricData();
        if (g1->getType() == GeometricData::Type::POLYGON)
        {
            Polygon* p1 = static_cast<Polygon*>(g1);

            if (p1->getDimensionType() == Polygon::DimensionType::TWO_D)
            {
                Polygon2D* p2d = static_cast<Polygon2D*>(p1);

                // check if the normal should be used by comparing it to
                // the face normals
                size_t count;
                const ID* faceIds = p2d->getRelevantFaces(*cs1.getTopologyFeature().get(), count);

                for (size_t i = 0; i < count; ++i)
                {
                    ID faceId = faceIds[i];
                    Eigen::Vector faceNormal = p2d->getFaceNormals()[faceId];

                    double angle = std::acos(faceNormal.dot(normal)) * 180.0 / M_PI;
                    if (faceNormal.dot(normal) < toleranceRad)
                    {
                        return false;
                    }
                }
            }
            else if (p1->getDimensionType() == Polygon::DimensionType::THREE_D)
            {
                Polygon3D* p3d = static_cast<Polygon3D*>(g1);

                // check if the normal should be used by comparing it to
                // the face normals
                size_t count;
                const ID* faceIds = p3d->getRelevantFaces(
                            *cs1.getTopologyFeature().get(), count);

                for (size_t i = 0; i < count; ++i)
                {
                    ID faceId = faceIds[i];
                    Eigen::Vector faceNormal = p3d->getOuterFaceNormals()[faceId];

                    double angle = std::acos(faceNormal.dot(normal)) * 180.0 / M_PI;
                    if (faceNormal.dot(normal) < toleranceRad)
                    {
                        return false;
                    }
                }
            }
        }
    }
    return true;
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
