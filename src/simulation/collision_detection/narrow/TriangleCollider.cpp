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
#include <scene/data/geometric/TopologyFeatureIterator.h>
#include <simulation/SimulationObject.h>


TriangleCollider::TriangleCollider(
        double collisionMargin,
        bool invertNormalsIfNecessary)
    : mInvertNormalsIfNecessary(invertNormalsIfNecessary)
{
    mMargin = collisionMargin;
    mMarginSquared = collisionMargin * collisionMargin;
}

TriangleCollider::~TriangleCollider()
{
}

void TriangleCollider::setCollisionMargin(double collisionMargin)
{
    mMargin = collisionMargin;
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
    GeometricData* gd = cs.getPointRef().getSimulationObject()->getGeometricData();
    if (gd->getType() == GeometricData::Type::POLYGON)
    {
        Polygon* poly =static_cast<Polygon*>(gd);
        Polygon2DTopology& topoTarget = poly->getAccessor2D()->getTopology2D();
        TopologyFeature& feature = *cs.getTopologyFeature().get();

        addPair(topoSource, face, ct.getSimulationObject().get(),
                topoTarget, feature, cs.getPointRef().getSimulationObject().get());
    }
}

void TriangleCollider::addTrianglePair(
        CollisionTriangle& ct1, CollisionTriangle& ct2)
{
    // source
    Polygon2DTopology& topoSource = ct1.getAccessor()->getTopology2D();
    TopologyFace& face1 = topoSource.getFace(ct1.getFaceId());

    // target
    Polygon2DTopology& topoTarget = ct2.getAccessor()->getTopology2D();
    TopologyFace& face2 = topoTarget.getFace(ct2.getFaceId());

    addPair(topoSource, face1, ct1.getSimulationObject().get(),
            topoTarget, face2, ct2.getSimulationObject().get());
}

void TriangleCollider::addSphereSpherePair(CollisionSphere& cs1, CollisionSphere& cs2)
{
    // target
    GeometricData* gd1 = cs1.getPointRef().getSimulationObject()->getGeometricData();
    GeometricData* gd2 = cs2.getPointRef().getSimulationObject()->getGeometricData();
    if (gd1->getType() == GeometricData::Type::POLYGON &&
        gd2->getType() == GeometricData::Type::POLYGON)
    {
        Polygon* poly1 =static_cast<Polygon*>(gd1);
        Polygon2DTopology& topoSource = poly1->getAccessor2D()->getTopology2D();
        TopologyFeature& feature1 = *cs1.getTopologyFeature().get();

        Polygon* poly2 =static_cast<Polygon*>(gd1);
        Polygon2DTopology& topoTarget = poly2->getAccessor2D()->getTopology2D();
        TopologyFeature& feature2 = *cs2.getTopologyFeature().get();

        addPair(topoSource, feature1, cs2.getPointRef().getSimulationObject().get(),
                topoTarget, feature2, cs2.getPointRef().getSimulationObject().get());
    }
}

void TriangleCollider::addPair(
        Polygon2DTopology& topoSource,
        TopologyFeature& featureSource,
        SimulationObject* soSource,
        Polygon2DTopology& topoTarget,
        TopologyFeature& featureTarget,
        SimulationObject* soTarget)
{
    TopologyFeatureIterator* sourceFeatures = getFeatures(topoSource, featureSource, mSourceTemp);
    TopologyFeatureIterator* targetFeatures = getFeatures(topoTarget, featureTarget, mTargetTemp);

    if (!sourceFeatures || !targetFeatures)
        return;

    for (size_t i = 0; i < sourceFeatures->getSize(); ++i, ++(*sourceFeatures))
    {
        TopologyFeature& fSource = **sourceFeatures;
        for (size_t j = 0; j < targetFeatures->getSize(); ++j, ++(*targetFeatures))
        {
            TopologyFeature& fTarget = **targetFeatures;
            if (fSource.getType() == TopologyFeature::Type::FACE &&
                fTarget.getType() == TopologyFeature::Type::VERTEX)
            {
                mFeaturePairsFV.insert(
                            FVPair(
                                static_cast<TopologyFace*>(&fSource),
                                static_cast<TopologyVertex*>(&fTarget)));
            }
            else if (fSource.getType() == TopologyFeature::Type::VERTEX &&
                     fTarget.getType() == TopologyFeature::Type::FACE)
            {
                mFeaturePairsFV.insert(
                            FVPair(
                                static_cast<TopologyFace*>(&fTarget),
                                static_cast<TopologyVertex*>(&fSource)));
            }
            else if (fSource.getType() == TopologyFeature::Type::EDGE &&
                     fTarget.getType() == TopologyFeature::Type::EDGE)
            {
                mFeaturePairsEE.insert(
                            EEPair(
                                static_cast<TopologyEdge*>(&fSource),
                                static_cast<TopologyEdge*>(&fTarget)));
            }
        }
        targetFeatures->reset();
    }

    sourceFeatures->reset();
    targetFeatures->reset();

    for (size_t i = 0; i < sourceFeatures->getSize(); ++i, ++(*sourceFeatures))
    {
        mFeatureToSoMap[&**sourceFeatures] = soSource;
    }

    for (size_t i = 0; i < targetFeatures->getSize(); ++i, ++(*targetFeatures))
    {
        mFeatureToSoMap[&**targetFeatures] = soTarget;
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
    SimulationObject* so1 = getSimulationObject(&v);
    SimulationObject* so2 = getSimulationObject(&f);

    // Collisions between non-polygons aren't supported
    if (so1->getGeometricData()->getType() != GeometricData::Type::POLYGON ||
        so2->getGeometricData()->getType() != GeometricData::Type::POLYGON)
    {
        return false;
    }

    Polygon* poly1 = static_cast<Polygon*>(so1->getGeometricData());
    Polygon* poly2 = static_cast<Polygon*>(so2->getGeometricData());

    Eigen::Vector& pos = poly1->getAccessor2D()->getPosition(v.getID());
    ID v1Index = v.getID(); // TODO: not correct

    Eigen::Vector inter; // projected point
    Eigen::Vector bary; // baryzentric coordinates
    bool isInside;

    Eigen::Vector& p1 = poly2->getPosition(f.getVertexIds()[0]);
    Eigen::Vector& p2 = poly2->getPosition(f.getVertexIds()[1]);
    Eigen::Vector& p3 = poly2->getPosition(f.getVertexIds()[2]);

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

        new (&collision) Collision(so1, so2,
                                   pos, inter,
                                   dir, 0.0,
                                   v1Index,
                                   v2Index,
                                   false);
        return true;
    }

    return false;
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

    Eigen::Vector x11 =
            poly1->getAccessor2D()->getPosition(e1.getVertexIds()[0]);
    Eigen::Vector x12 =
            poly1->getAccessor2D()->getPosition(e1.getVertexIds()[1]);

    Eigen::Vector x21 =
            poly2->getAccessor2D()->getPosition(e2.getVertexIds()[0]);
    Eigen::Vector x22 =
            poly2->getAccessor2D()->getPosition(e2.getVertexIds()[1]);

    // Reduce the affected area so they don't overlap with vertex-triangle collisions.
    // There are porbably faster ways of doing this.
    Eigen::Vector x1dir = (x12 - x11).normalized();
    Eigen::Vector x2dir = (x22 - x21).normalized();

    x11 += mMargin * x1dir;
    x12 -= mMargin * x1dir;

    x21 += mMargin * x2dir;
    x22 -= mMargin * x2dir;

    Eigen::Vector3d inter1; // projected point 1
    Eigen::Vector3d inter2; // projected point 2
    Eigen::Vector2d bary; // baryzentric coordinates
    bool isInside;

    bool ok = MathUtils::projectEdgeOnEdge(
                x11, x12, x21, x22, inter1, inter2, bary, isInside);

    if (ok && isInside && (inter1 - inter2).squaredNorm() < mMarginSquared)
    {
        Eigen::Vector dir = (inter1 - inter2).normalized();

        if (mInvertNormalsIfNecessary)
        {
            bool isInside =
                    poly1->isInside(e1, inter2) ||
                    poly2->isInside(e2, inter1);
            if (isInside)
                dir = -dir;
        }

        new (&collision) Collision(
                    so1, so2,
                    inter1, inter2, dir, 0.0, 0, 0, false);
        return true;
    }

    return false;
}

TopologyFeatureIterator* TriangleCollider::getFeatures(
        Polygon2DTopology& topo,
        TopologyFeature& feature,
        IteratorPair& temp)
{
    std::vector<TopologyFeature*> features;

    // extract relevant features
    switch(feature.getType())
    {
    case TopologyFeature::Type::FACE:
    {
        // face
        TopologyFace* face = static_cast<TopologyFace*>(&feature);

        class FaceFeatureIterator : public TopologyFeatureIterator
        {
        public:
            FaceFeatureIterator(TopologyFace& _face, Polygon2DTopology& _topo)
                : face(_face)
                , topo(_topo)
            {
                index = 0;
            }

            TopologyFeatureIterator& operator++() override
            {
                ++index;
                return *this;
            }

            TopologyFeature& operator*() override
            {
                if (index == 0)
                {
                    return face;
                }
                else if (index < 4)
                {
                    return topo.getEdge(face.getEdgeIds()[index - 1]);
                }
                else
                {
                    return topo.getVertex(face.getVertexIds()[index - 4]);
                }
            }

            void reset() override
            {
                index = 0;
            }

            size_t getSize() override
            {
                return 7;
            }

            TopologyFace& face;
            Polygon2DTopology& topo;

            size_t index;
        };

        if (!temp.mFaceIteratorTemp)
            temp.mFaceIteratorTemp = new FaceFeatureIterator(*face, topo);
        else
            new (temp.mFaceIteratorTemp) FaceFeatureIterator(*face, topo);
        return temp.mFaceIteratorTemp;
    }
    case TopologyFeature::Type::VERTEX:
    {
        TopologyVertex* vertex = static_cast<TopologyVertex*>(&feature);

        class VertexFeatureIterator : public TopologyFeatureIterator
        {
        public:
            VertexFeatureIterator(TopologyVertex& _vertex)
                : vertex(_vertex)
            {
            }

            TopologyFeatureIterator& operator++() override
            {
                return *this;
            }

            TopologyFeature& operator*() override
            {
                return vertex;
            }

            void reset() override
            {
            }

            size_t getSize() override
            {
                return 1;
            }

            TopologyVertex& vertex;
        };

        if (!temp.mVertexIteratorTemp)
            temp.mVertexIteratorTemp = new VertexFeatureIterator(*vertex);
        else
            new (temp.mVertexIteratorTemp) VertexFeatureIterator(*vertex);
        return temp.mVertexIteratorTemp;
    }
    case TopologyFeature::Type::EDGE:
    case TopologyFeature::Type::CELL:
        std::cout << "Warning: unsupported feature type.\n";
    }

    return nullptr;
}
