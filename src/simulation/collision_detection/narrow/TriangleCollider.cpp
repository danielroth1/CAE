#include "Collision.h"
#include "CollisionSphere.h"
#include "CollisionTriangle.h"
#include "TriangleCollider.h"

#include <iostream>
#include <math/MathUtils.h>
#include <math/MathUtils.h>
#include <scene/data/GeometricData.h>
#include <scene/data/geometric/Polygon.h>
#include <scene/data/geometric/Polygon2DTopology.h>
#include <simulation/SimulationObject.h>


TriangleCollider::TriangleCollider(
        double collisionMargin,
        bool invertNormalsIfNecessary)
    : mInvertNormalsIfNecessary(invertNormalsIfNecessary)
{
    mMarginSquared = collisionMargin * collisionMargin;
}

TriangleCollider::~TriangleCollider()
{

}

void TriangleCollider::setCollisionMargin(double collisionMargin)
{
    mMarginSquared = collisionMargin * collisionMargin;
}

void TriangleCollider::setInvertNormalsIfNecessary(bool invertNormals)
{
    mInvertNormalsIfNecessary = invertNormals;
}

void TriangleCollider::addTriangleSpherePair(
        CollisionTriangle& ct, CollisionSphere& cs)
{
    // source
    Polygon2DTopology& topoSource = ct.getAccessor()->getTopology2D();
    TopologyFace& face = topoSource.getFace(ct.getFaceId());

    // target
    Polygon2DTopology& topoTarget = ct.getAccessor()->getTopology2D();
    TopologyFeature& feature = *cs.getTopologyFeature().get();

    addPair(topoSource, face, ct.getSimulationObject().get(),
            topoTarget, feature, cs.getPointRef().getSimulationObject().get());
}

void TriangleCollider::addTrianglePair(
        CollisionTriangle& ct1, CollisionTriangle& ct2)
{
    // source
    Polygon2DTopology& topoSource = ct1.getAccessor()->getTopology2D();
    TopologyFace& face1 = topoSource.getFace(ct1.getFaceId());

    // target
    Polygon2DTopology& topoTarget = ct2.getAccessor()->getTopology2D();
    TopologyFace& face2 = topoSource.getFace(ct2.getFaceId());

    addPair(topoSource, face1, ct1.getSimulationObject().get(),
            topoTarget, face2, ct1.getSimulationObject().get());
}

void TriangleCollider::addPair(
        Polygon2DTopology& topoSource,
        TopologyFeature& featureSource,
        SimulationObject* soSource,
        Polygon2DTopology& topoTarget,
        TopologyFeature& featureTarget,
        SimulationObject* soTarget)
{
    std::vector<TopologyFeature*> sourceFeatures = getFeatures(topoSource, featureSource);
    std::vector<TopologyFeature*> targetFeatures = getFeatures(topoTarget, featureTarget);

    for (TopologyFeature* fSource : sourceFeatures)
    {
        for (TopologyFeature* fTarget : targetFeatures)
        {
            if (fSource->getType() == TopologyFeature::Type::FACE &&
                fSource->getType() == TopologyFeature::Type::VERTEX)
            {
                mFeaturePairsFV.insert(
                            std::make_pair(
                                static_cast<TopologyFace*>(fSource),
                                static_cast<TopologyVertex*>(fTarget)));
            }
            else if (fSource->getType() == TopologyFeature::Type::EDGE &&
                     fSource->getType() == TopologyFeature::Type::EDGE)
             {
                 mFeaturePairsEE.insert(
                             std::make_pair(
                                 static_cast<TopologyEdge*>(fSource),
                                 static_cast<TopologyEdge*>(fTarget)));
             }
        }
    }

    for (TopologyFeature* fSource : sourceFeatures)
    {
        mFeatureToSoMap[fSource] = soSource;
    }

    for (TopologyFeature* fTarget : targetFeatures)
    {
        mFeatureToSoMap[fTarget] = soTarget;
    }
}

void TriangleCollider::clear()
{
    mFeaturePairsFV.clear();
    mFeaturePairsEE.clear();
    mFeatureToSoMap.clear();
}

void TriangleCollider::collide(std::vector<Collision>& collisions)
{
    // Face-Vertex
    for (const std::pair<TopologyFace*, TopologyVertex*>& pair : mFeaturePairsFV)
    {
        Collision c;
        bool collides = collide(*pair.first, *pair.second, c);
        if (collides)
        {
            collisions.push_back(c);
        }
    }

    // Edge-Edge
    for (const std::pair<TopologyEdge*, TopologyEdge*>& pair : mFeaturePairsEE)
    {
        Collision c;
        bool collides = collide(*pair.first, *pair.second, c);
        if (collides)
        {
            collisions.push_back(c);
        }
    }
}

bool TriangleCollider::collide(
        TopologyFace& f, TopologyVertex& v, Collision& collision)
{
    SimulationObject* so1 = getSimulationObject(&f);
    SimulationObject* so2 = getSimulationObject(&v);

    // Collisions between non-polygons aren't supported
    if (so1->getGeometricData()->getType() != GeometricData::Type::POLYGON ||
        so2->getGeometricData()->getType() != GeometricData::Type::POLYGON)
    {
        return false;
    }

    Polygon* poly1 = static_cast<Polygon*>(so1->getGeometricData());
    Polygon* poly2 = static_cast<Polygon*>(so2->getGeometricData());

    Eigen::Vector& pos = so2->getPosition(v.getID());
    ID v1Index = f.getID(); // TODO: not correct

    Eigen::Vector inter; // projected point
    Eigen::Vector bary; // baryzentric coordinates
    bool isInside;

    Eigen::Vector& p1 = poly1->getPosition(f.getVertexIds()[0]);
    Eigen::Vector& p2 = poly1->getPosition(f.getVertexIds()[1]);
    Eigen::Vector& p3 = poly1->getPosition(f.getVertexIds()[2]);

    bool ok = MathUtils::projectPointOnTriangle(
                p1, p2, p3,
                pos,
                inter, bary, isInside);

    if (ok && (pos - inter).squaredNorm() < mMarginSquared)
    {
        // Determin collsion normal -> triangle normal...
        Eigen::Vector dir;
        if (isInside)
        {
            dir = poly2->getAccessor2D()->getFaceNormals()[f.getID()];
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

        ID v2Index = v.getID();

        new (&collision) Collision(so1->shared_from_this(),
                                   so2->shared_from_this(),
                                   pos, inter, dir, 0.0,
                                   v1Index,
                                   v2Index,
                                   false);
        return true;
    }
}

bool TriangleCollider::collide(
        TopologyEdge& e1, TopologyEdge& e2, Collision& collision)
{
    SimulationObject* so1 = getSimulationObject(&e1);
    SimulationObject* so2 = getSimulationObject(&e2);

    // Collisions between non-polygons aren't supported
    if (so1->getGeometricData()->getType() != GeometricData::Type::POLYGON ||
        so2->getGeometricData()->getType() != GeometricData::Type::POLYGON)
    {
        return false;
    }

    Polygon* poly1 = static_cast<Polygon*>(so1->getGeometricData());
    Polygon* poly2 = static_cast<Polygon*>(so2->getGeometricData());

    const Eigen::Vector& x11 =
            poly1->getAccessor2D()->getPosition(e1.getVertexIds()[0]);
    const Eigen::Vector& x12 =
            poly1->getAccessor2D()->getPosition(e1.getVertexIds()[2]);

    const Eigen::Vector& x21 =
            poly2->getAccessor2D()->getPosition(e2.getVertexIds()[0]);
    const Eigen::Vector& x22 =
            poly2->getAccessor2D()->getPosition(e2.getVertexIds()[2]);

    Eigen::Vector3d inter1; // projected point 1
    Eigen::Vector3d inter2; // projected point 2
    Eigen::Vector2d bary; // baryzentric coordinates
    bool isInside;

    bool ok = MathUtils::projectEdgeOnEdge(
                x11, x12, x21, x22, inter1, inter2, bary, isInside);

    if (ok && (inter1 - inter2).squaredNorm() < mMarginSquared)
    {
        // Determin collsion normal -> triangle normal...
//            Eigen::Vector dir = ct2.getAccessor()->getFaceNormals()[ct2.getFaceId()];

        Eigen::Vector dir = (inter1 - inter2).normalized();

        if (mInvertNormalsIfNecessary)
        {
            bool isInside = true;
            isInside =
                    poly1->isInside(e1, inter2);
            isInside |=
                    poly2->isInside(e2, inter1);
            if (isInside)
                dir = -dir;
        }

        new (&collision) Collision(
                    so1->shared_from_this(), so2->shared_from_this(),
                    inter1, inter2, dir, 0.0, 0, 0, false);
        return true;
    }

    return false;
}

std::vector<TopologyFeature*> TriangleCollider::getFeatures(
        Polygon2DTopology& topo, TopologyFeature& feature)
{
    std::vector<TopologyFeature*> features;

    // extract relevant features
    switch(feature.getType())
    {
    case TopologyFeature::Type::FACE:
    {
        // face
        TopologyFace* face = static_cast<TopologyFace*>(&feature);
        features.push_back(face);

        // edge
        for (ID id : face->getEdgeIds())
        {
            features.push_back(&topo.getEdge(id));
        }

        // vertex
        for (ID id : face->getVertexIds())
        {
            features.push_back(&topo.getVertex(id));
        }

        break;
    }
    case TopologyFeature::Type::VERTEX:
    {
        TopologyVertex* vertex = static_cast<TopologyVertex*>(&feature);

        features.push_back(vertex);

        break;
    }
    case TopologyFeature::Type::EDGE:
    case TopologyFeature::Type::CELL:
        std::cout << "Warning: unsupported feature type.\n";
    }

    return features;
}
