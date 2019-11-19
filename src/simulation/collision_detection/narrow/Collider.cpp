#include "Collider.h"
#include "CollisionObject.h"
#include "CollisionSphere.h"
#include "CollisionTriangle.h"

#include <iostream>
#include <new>

#include <scene/data/geometric/Polygon.h>
#include <scene/data/geometric/Polygon2D.h>
#include <scene/data/geometric/Polygon2DData.h>
#include <scene/data/geometric/Polygon2DTopology.h>
#include <scene/data/geometric/Polygon3D.h>
#include <scene/data/geometric/PolygonTopology.h>
#include <scene/data/geometric/TopologyFeature.h>

#include <simulation/SimulationObject.h>

#include <scene/data/GeometricData.h>

#include <math/MathUtils.h>

Collider::Collider()
    : mInvertNormalsIfNecessary(false)
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
    switch(co2.getType())
    {
    case CollisionObject::Type::SPHERE:
        return collides(co1, *static_cast<CollisionSphere*>(&co2), collisionReturnValue);
    case CollisionObject::Type::TRIANGLE:
        return collides(co1, *static_cast<CollisionTriangle*>(&co2), collisionReturnValue);
    }
    return false;
}

bool Collider::collides(
        CollisionObject& co,
        CollisionSphere& cs,
        Collision& collisionReturnValue)
{
    switch(co.getType())
    {
    case CollisionObject::Type::SPHERE:
        return collides(*static_cast<CollisionSphere*>(&co), cs, collisionReturnValue);
    case CollisionObject::Type::TRIANGLE:
    {
        bool result = collides(cs, *static_cast<CollisionTriangle*>(&co), collisionReturnValue);
        return result;
    }
    }
    return false;
}

bool Collider::collides(
        CollisionObject& co,
        CollisionTriangle& ct,
        Collision& collisionReturnValue)
{
    switch(co.getType())
    {
    case CollisionObject::Type::SPHERE:
        return collides(*static_cast<CollisionSphere*>(&co), ct, collisionReturnValue);
    case CollisionObject::Type::TRIANGLE:
        return collides(*static_cast<CollisionTriangle*>(&co), ct, collisionReturnValue);
    }
    return false;
}

bool Collider::collides(
        CollisionSphere& cs1,
        CollisionSphere& cs2,
        Collision& collisionReturnValue)
{
    Eigen::Vector p1 = cs1.getPosition();
    Eigen::Vector p2 = cs2.getPosition();

    double radi = cs1.getRadius() + cs2.getRadius();
    bool returnValue = (p1 - p2).squaredNorm() < radi * radi;

    if (!returnValue)
        return returnValue; // There is no collision

//    if (returnValue)
//        std::cout << "pos1 = " << p1.transpose() <<
//                     ", pos2 = " << p1.transpose() <<
//                     ", radius1 = " << cs1.getRadius() <<
//                     ", radius2 = " << cs2.getRadius() << "\n";

    // Contact normal is always directed from the second body to the first one.
    Eigen::Vector normal = (p1 - p2).normalized();
    Eigen::Vector pointA = p1 - cs1.getRadius() * normal;
    Eigen::Vector pointB = p2 + cs2.getRadius() * normal;

    double depth = (pointA - pointB).norm();
    bool isIn = false;

    if (mInvertNormalsIfNecessary)
    {
        bool isIn1 = isInside(cs1, cs2);
        bool isIn2 = isInside(cs2, cs1);

        // if both are inside, the normal can be safely reverted and it will
        // be correct.
        if (isIn1 && isIn2)
        {
            normal *= -1;
        }
        else if (isIn1 || isIn2)
        {
            // in some cases reverting the normal doesn't work (not sure why exactly).
            // perform a check if both ascending face normals and contact normal
            // point in the same (or for the other object opposite) direction. If not
            // remove the contact.

            bool passesFaceNormalCheck1 = true;
            if (isIn1)
                passesFaceNormalCheck1 = passesFaceNormalTest(cs1, -normal);

            bool passesFaceNormalCheck2 = true;
            if (isIn2)
                passesFaceNormalCheck2 = passesFaceNormalTest(cs2, normal);

            if (!passesFaceNormalCheck1 || !passesFaceNormalCheck2)
            {
                return false;
            }
        }
    }

    // Calculate normal
    // TODO: Here the simulation object is needed. Thats why CollisionSphere
    // has to know about it. To connect the collision to the simulation object.
    // Do this differently.
    // Collider could have a map from GeometricData to SimulationObject.
    new (&collisionReturnValue) Collision(cs1.getPointRef().getSimulationObject(),
                                          cs2.getPointRef().getSimulationObject(),
                                          pointA, pointB, normal, depth,
                                          cs1.getVertexIndex(),
                                          cs2.getVertexIndex(),
                                          isIn);

    return returnValue;
}

bool Collider::collides(
        CollisionSphere& cs,
        CollisionTriangle& ct,
        Collision& collisionReturnValue)
{
    // sphere-triangle
    Eigen::Vector spherePos = cs.getPosition();

    Eigen::Vector inter; // projected point
    Eigen::Vector bary; // baryzentric coordinates
    bool isInside;
    MathUtils::projectPointOnTriangle(
                ct.getP1(), ct.getP2(), ct.getP3(),
                spherePos,
                inter, bary, isInside);

    if ((inter - spherePos).squaredNorm() < cs.getRadius() * cs.getRadius())
    {
        Eigen::Vector dir = (spherePos - inter).normalized();
        Eigen::Vector pointB = spherePos - cs.getRadius() * dir;

        ID index = 0;
        if (bary(1) > bary(0) && bary(1) > bary(2))
            index = 1;
        else if (bary(2) > bary(0) && bary(2) > bary(1))
            index = 2;

        ID triIndex = ct.getFace()[index];

        if (mInvertNormalsIfNecessary)
        {
            Eigen::Vector faceNormal = ct.getAccessor()->getFaceNormals()[ct.getFaceId()];
            if (dir.dot(faceNormal) < 0)
                dir = -dir;
        }

        new (&collisionReturnValue) Collision(cs.getPointRef().getSimulationObject(),
                                              ct.getSimulationObject(),
                                              pointB, inter, dir, 0.0,
                                              cs.getVertexIndex(),
                                              triIndex,
                                              false);

        return true;
    }

    return false;
}

bool Collider::collides(
        CollisionTriangle& ct1,
        CollisionTriangle& ct2,
        Collision& collisionReturnValue)
{
    bool collision = collidesTriangle(ct1, ct2, collisionReturnValue);
    if (!collision)
    {
        collision = collidesTriangle(ct2, ct1, collisionReturnValue);
    }
    return collision;
}

bool Collider::collidesTriangle(
        CollisionTriangle& ct1,
        CollisionTriangle& ct2,
        Collision& collisionReturnValue)
{
    double margin = 5e-2;
    double marginSquared = margin * margin;

    bool result = collidesTrianglesPair(ct1, ct2, marginSquared, collisionReturnValue);
    if (!result)
    {
        result = collidesTrianglesPair(ct2, ct1, marginSquared, collisionReturnValue);

        if (!result)
        {
            result = collidesEdgesPair(ct1, ct2, marginSquared, collisionReturnValue);
        }
    }
    return result;
}

bool Collider::collidesTrianglesPair(
        CollisionTriangle& ct1,
        CollisionTriangle& ct2,
        double marginSquared,
        Collision& collisionReturnValue)
{
    // point triangle
    Eigen::Vector& p21 = ct2.getP1();
    Eigen::Vector& p22 = ct2.getP2();
    Eigen::Vector& p23 = ct2.getP3();
    for (ID i = 0; i < 3; ++i)
    {
        Eigen::Vector& pos = ct1.getPosition(i);
        ID v1Index = ct1.getFace()[i];

        Eigen::Vector inter; // projected point
        Eigen::Vector bary; // baryzentric coordinates
        bool isInside;

        bool ok = MathUtils::projectPointOnTriangle(
                    p21, p22, p23,
                    pos,
                    inter, bary, isInside);

        if (ok && (pos - inter).squaredNorm() < marginSquared)
        {
            // Determin collsion normal -> triangle normal...
            Eigen::Vector dir;
            if (isInside)
            {
                dir = ct2.getAccessor()->getFaceNormals()[ct2.getFaceId()];
            }
            else
            {
                dir = (pos - inter).normalized();
//                if (dir.dot(ct2.getAccessor()->getFaceNormals()[ct2.getFaceId()]) < 0)
//                {
//                    dir = -dir;
//                }
            }

            ID index = 0;
            if (bary(1) > bary(0) && bary(1) > bary(2))
                index = 1;
            else if (bary(2) > bary(0) && bary(2) > bary(1))
                index = 2;

            ID v2Index = ct2.getFace()[index];

            new (&collisionReturnValue) Collision(ct1.getSimulationObject(),
                                                  ct2.getSimulationObject(),
                                                  pos, inter, dir, 0.0,
                                                  v1Index,
                                                  v2Index,
                                                  false);
            return true;
        }
    }

    return false;
}

bool Collider::collidesEdgesPair(CollisionTriangle& ct1,
                                 CollisionTriangle& ct2,
                                 double marginSquared,
                                 Collision& collisionReturnValue)
{
    // edge-edge
    for (ID i = 0; i < 3; ++i)
    {
        const Eigen::Vector* x11;
        const Eigen::Vector* x12;
        if (i == 0)
        {
            x11 = &ct1.getP1();
            x12 = &ct1.getP2();
        }
        else if (i == 1)
        {
            x11 = &ct1.getP2();
            x12 = &ct1.getP3();
        }
        else
        {
            x11 = &ct1.getP3();
            x12 = &ct1.getP1();
        }

        for (ID j = 0; j < 3; ++j)
        {
            const Eigen::Vector* x21;
            const Eigen::Vector* x22;
            if (j == 0)
            {
                x21 = &ct2.getP1();
                x22 = &ct2.getP2();
            }
            else if (j == 1)
            {
                x21 = &ct2.getP2();
                x22 = &ct2.getP3();
            }
            else
            {
                x21 = &ct2.getP3();
                x22 = &ct2.getP1();
            }

            Eigen::Vector3d inter1; // projected point 1
            Eigen::Vector3d inter2; // projected point 2
            Eigen::Vector2d bary; // baryzentric coordinates
            bool isInside;

            bool ok = MathUtils::projectEdgeOnEdge(
                        *x11, *x12, *x21, *x22, inter1, inter2, bary, isInside);

            if (ok && (inter1 - inter2).squaredNorm() < marginSquared)
            {
                // Determin collsion normal -> triangle normal...
    //            Eigen::Vector dir = ct2.getAccessor()->getFaceNormals()[ct2.getFaceId()];

                Eigen::Vector dir = (inter1 - inter2).normalized();

                if (mInvertNormalsIfNecessary)
                {
                    bool isInside = true;
                    isInside =
                            ct1.getAccessor()->getPolygon()->isInside(
                                ct1.getAccessor()->getTopology2D().getFace(ct1.getFaceId()),
                               inter2);
                    isInside |=
                            ct2.getAccessor()->getPolygon()->isInside(
                                ct2.getAccessor()->getTopology2D().getFace(ct2.getFaceId()),
                               inter1);
                    if (isInside)
                        dir = -dir;
                }

                new (&collisionReturnValue) Collision(ct1.getSimulationObject(),
                                                      ct2.getSimulationObject(),
                                                      inter1, inter2, dir, 0.0,
                                                      0,
                                                      0,
                                                      false);
                return true;
            }
        }
    }
    return false;
}

bool Collider::isInside(CollisionSphere& cs1, CollisionSphere& cs2)
{
    bool isInside = true;
    if (cs1.getTopologyFeature() != nullptr)
    {
        GeometricData* g1 = cs1.getPointRef().getGeometricData();
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
    double toleranceAngleDegree = 0.0;
    double toleranceRad = toleranceAngleDegree / 180.0 * M_PI;

    if (cs1.getTopologyFeature() != nullptr)
    {
        GeometricData* g1 = cs1.getPointRef().getGeometricData();
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
