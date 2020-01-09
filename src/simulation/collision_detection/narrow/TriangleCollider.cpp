#include "Collision.h"
#include "CollisionSphere.h"
#include "CollisionTriangle.h"
#include "TriangleCollider.h"

#include <cassert>
#include <iostream>
#include <math/MathUtils.h>
#include <math/MathUtils.h>
#include <scene/data/GeometricData.h>
#include <scene/data/geometric/Polygon.h>
#include <scene/data/geometric/Polygon2DTopology.h>
#include <scene/data/geometric/Polygon3DTopology.h>
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

        addPair(topoSource, face, topoTarget, feature);
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

    assert(ct1.getAccessor()->getPolygon() == mPoly1 && ct2.getAccessor()->getPolygon()== mPoly2);

    addPair(topoSource, face1, topoTarget, face2);
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

        addPair(topoSource, feature1, topoTarget, feature2);
    }
}

void TriangleCollider::addPair(
        Polygon2DTopology& topoSource,
        TopologyFeature& featureSource,
        Polygon2DTopology& topoTarget,
        TopologyFeature& featureTarget)
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
                FVPair pair(static_cast<TopologyFace*>(&fSource),
                            static_cast<TopologyVertex*>(&fTarget));
                if (mFeaturePairsFVSet.find(pair) == mFeaturePairsFVSet.end())
                {
                    mFeaturePairsFVSet.insert(pair);
                    mFeaturePairsFV.push_back(pair);
                }
            }
            else if (fSource.getType() == TopologyFeature::Type::VERTEX &&
                     fTarget.getType() == TopologyFeature::Type::FACE)
            {
                FVPair fvPair(
                    static_cast<TopologyFace*>(&fTarget),
                    static_cast<TopologyVertex*>(&fSource));
                if (mFeaturePairsFVSet.find(fvPair) == mFeaturePairsFVSet.end())
                {
                    mFeaturePairsFVSet.insert(fvPair);
                    VFPair pair(static_cast<TopologyVertex*>(&fSource),
                                static_cast<TopologyFace*>(&fTarget));
                    mFeaturePairsVF.push_back(pair);
                }
            }
            else if (fSource.getType() == TopologyFeature::Type::EDGE &&
                     fTarget.getType() == TopologyFeature::Type::EDGE)
            {
                // Avoid duplications, e.g. (a, b) and (b, a), by ordering
                // the pair.
                EEPair pair(static_cast<TopologyEdge*>(&fSource),
                            static_cast<TopologyEdge*>(&fTarget));
                if (&fSource < &fTarget)
                {
                    if (mFeaturePairsEESet.find(pair) == mFeaturePairsEESet.end())
                    {
                        mFeaturePairsEESet.insert(pair);
                        mFeaturePairsEE.push_back(pair);
                    }
                }
                else
                {
                    EEPair eePair(static_cast<TopologyEdge*>(&fTarget),
                                  static_cast<TopologyEdge*>(&fSource));

                    if (mFeaturePairsEESet.find(eePair) == mFeaturePairsEESet.end())
                    {
                        mFeaturePairsEESet.insert(eePair);
                        mFeaturePairsEE.push_back(pair);
                    }

                }
            }
        }
        targetFeatures->reset();
    }

    sourceFeatures->reset();
    targetFeatures->reset();
}

void TriangleCollider::prepare(
        Polygon* poly1, Polygon* poly2,
        SimulationObject* so1, SimulationObject* so2,
        MeshInterpolatorFEM* interpolator1, MeshInterpolatorFEM* interpolator2)
{
    mPoly1 = poly1;
    mPoly2 = poly2;
    mSo1 = so1;
    mSo2 = so2;
    mInterpolator1 = interpolator1;
    mInterpolator2 = interpolator2;

    clear();
}

void TriangleCollider::clear()
{
    mFeaturePairsFVSet.clear();
    mFeaturePairsEESet.clear();
    mFeaturePairsFV.clear();
    mFeaturePairsVF.clear();
    mFeaturePairsEE.clear();
}

void TriangleCollider::collide(std::vector<Collision>& collisions)
{
    // Vertex-Face
    for (const std::pair<TopologyVertex*, TopologyFace*>& pair : mFeaturePairsVF)
    {
        Collision c;
        bool collides = collide(*pair.second, *pair.first, true, c);
        if (collides)
        {
            collisions.push_back(c);
        }
    }

    // Face-Vertex
    for (const std::pair<TopologyFace*, TopologyVertex*>& pair : mFeaturePairsFV)
    {
        Collision c;
        bool collides = collide(*pair.first, *pair.second, false, c);
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
        TopologyFace& f,
        TopologyVertex& v,
        bool reverted,
        Collision& collision)
{
    SimulationObject* so1;
    SimulationObject* so2;
    Polygon* poly1;
    Polygon* poly2;
    MeshInterpolatorFEM* interpolator1;
    MeshInterpolatorFEM* interpolator2;
    if (reverted)
    {
        so1 = mSo1;
        so2 = mSo2;
        poly1 = mPoly1;
        poly2 = mPoly2;
        interpolator1 = mInterpolator1;
        interpolator2 = mInterpolator2;
    }
    else
    {
        so1 = mSo2;
        so2 = mSo1;
        poly1 = mPoly2;
        poly2 = mPoly1;
        interpolator1 = mInterpolator2;
        interpolator2 = mInterpolator1;
    }

    // Collisions between non-polygons aren't supported
    if (so1->getGeometricData()->getType() != GeometricData::Type::POLYGON ||
        so2->getGeometricData()->getType() != GeometricData::Type::POLYGON)
    {
        return false;
    }

    Eigen::Vector& pos = poly1->getAccessor2D()->getPosition(v.getID());

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

        ID v1Index = 0;
        if (poly1->getDimensionType() == Polygon::DimensionType::THREE_D)
        {
            Polygon3DTopology* t3 = static_cast<Polygon3DTopology*>(&poly1->getTopology());
            v1Index = t3->getOuterVertexIds()[v.getID()];
        }

        ID v2Index = 0;
        if (poly2->getDimensionType() == Polygon::DimensionType::THREE_D)
        {
            Polygon3DTopology* t3 = static_cast<Polygon3DTopology*>(&poly2->getTopology());

            ID index = 0;
            if (bary(1) > bary(0) && bary(1) > bary(2))
                index = 1;
            else if (bary(2) > bary(0) && bary(2) > bary(1))
                index = 2;

            v2Index = t3->getFace(t3->getOuterFaceIds()[f.getID()]).getVertexIds()[index];
//            v1Index = t3->getOuterFaces()[f.getID()].getVertexIds()[index];
//            v1Index = t3->getOuterVertexIds()[f.getVertexIds()[index]];
        }

        new (&collision) Collision(so1, so2,
                                   pos, inter,
                                   dir, 0.0,
                                   v1Index,
                                   v2Index,
                                   false);

        ID elementId;
        fillBarycentricCoordinates(poly1, v, interpolator1, elementId, collision.getBarycentricCoordiantesA());
        collision.setElementIdA(elementId);

        fillBarycentricCoordinates(poly2, f, bary, interpolator2, elementId, collision.getBarycentricCoordiantesB());
        collision.setElementIdB(elementId);
        return true;
    }

    return false;
}

bool TriangleCollider::collide(
        TopologyEdge& e1,
        TopologyEdge& e2,
        Collision& collision)
{
    SimulationObject* so1 = mSo1;
    SimulationObject* so2 = mSo2;
    MeshInterpolatorFEM* interpolator1 = mInterpolator1;
    MeshInterpolatorFEM* interpolator2 = mInterpolator2;
    Polygon* poly1 = mPoly1;
    Polygon* poly2 = mPoly2;

    // Collisions between non-polygons aren't supported
    if (so1->getGeometricData()->getType() != GeometricData::Type::POLYGON ||
        so2->getGeometricData()->getType() != GeometricData::Type::POLYGON)
    {
        return false;
    }

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

        ID v1Index = 0;
        ID v2Index = 0;

        if (poly1->getDimensionType() == Polygon::DimensionType::THREE_D)
        {
            Polygon3DTopology* t3 = static_cast<Polygon3DTopology*>(&poly1->getTopology());

            if (bary(0) < 0.5)
                v1Index = t3->getOuterEdges()[e1.getID()].getVertexIds()[0];
            else
                v1Index = t3->getOuterEdges()[e1.getID()].getVertexIds()[1];

        }

        if (poly2->getDimensionType() == Polygon::DimensionType::THREE_D)
        {
            Polygon3DTopology* t3 = static_cast<Polygon3DTopology*>(&poly2->getTopology());
            if (bary(1) < 0.5)
                v2Index = t3->getOuterEdges()[e2.getID()].getVertexIds()[0];
            else
                v2Index = t3->getOuterEdges()[e2.getID()].getVertexIds()[0];
        }

        new (&collision) Collision(
                    so1, so2,
                    inter1, inter2, dir, 0.0, v1Index, v2Index, false);

        ID elementId;
        fillBarycentricCoordinates(poly1, e1, bary(0), interpolator1, elementId, collision.getBarycentricCoordiantesA());
        collision.setElementIdA(elementId);

        fillBarycentricCoordinates(poly2, e2, bary(1), interpolator2, elementId, collision.getBarycentricCoordiantesB());
        collision.setElementIdB(elementId);

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

            TopologyFeatureIterator& operator++() override final
            {
                ++index;
                return *this;
            }

            TopologyFeature& operator*() override final
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

            void reset() override final
            {
                index = 0;
            }

            size_t getSize() override final
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

            TopologyFeatureIterator& operator++() override final
            {
                return *this;
            }

            TopologyFeature& operator*() override final
            {
                return vertex;
            }

            void reset() override final
            {
            }

            size_t getSize() override final
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

bool TriangleCollider::fillBarycentricCoordinates(
        Polygon* poly,
        TopologyFace& f,
        const Vector& bary,
        MeshInterpolatorFEM* interpolator,
        ID& elementIdOut,
        Eigen::Vector4d& baryOut)
{
    baryOut = Eigen::Vector4d::Zero();
    if (poly->getDimensionType() == Polygon::DimensionType::THREE_D)
    {
        Polygon3DTopology* topo = static_cast<Polygon3DTopology*>(&poly->getTopology());
        // obtain cell id
        // face is of outer topology2d
        // required cell is w.r.t. face of corresponding Topology3D
        TopologyFace& face3D = topo->getFace(topo->getOuterFaceIds()[f.getID()]);
        if (face3D.getCellIds().size() > 0)
        {
            ID cellId = face3D.getCellIds()[0];
            TopologyCell& cell = topo->getCells()[cellId];

            elementIdOut = cellId;

            // Map cell vertex ids to face vertex ids.
            for (size_t i = 0; i < 3; ++i)
            {
                for (size_t j = 0; j < 4; ++j)
                {
                    if (face3D.getVertexIds()[i] == cell.getVertexIds()[j])
                    {
                        baryOut[j] = bary[static_cast<Eigen::Index>(i)];
                        break;
                    }
                }
            }

            return true;
        }

        std::cout << "cell ids empty\n";
    }
    else if (poly->getDimensionType() == Polygon::DimensionType::TWO_D)
    {
        if (interpolator)
        {
            ID cellId;
            baryOut = interpolator->calculateBary3(f.getID(), bary, cellId);
            elementIdOut = cellId;
            return true;
        }
    }
    return false;
}

bool TriangleCollider::fillBarycentricCoordinates(
        Polygon* poly,
        TopologyEdge& e,
        double bary,
        MeshInterpolatorFEM* interpolator,
        ID& elementIdOut,
        Eigen::Vector4d& baryOut)
{
    baryOut = Eigen::Vector4d::Zero();
    if (poly->getDimensionType() == Polygon::DimensionType::THREE_D)
    {
        Polygon3DTopology* topo = static_cast<Polygon3DTopology*>(&poly->getTopology());

        // obtain cell id
        // face is of outer topology2d
        // required cell is w.r.t. face of corresponding Topology3D
//        TopologyEdge& edge3D = topo->getOuterEdges()[e.getID()];
        TopologyEdge& edge3D = topo->getEdge(topo->getOuterEdgeIds()[e.getID()]);
        if (!edge3D.getCellIds().empty())
        {
            ID cellId = edge3D.getCellIds()[0];
            TopologyCell& cell = topo->getCells()[cellId];

            elementIdOut = cellId;

            // Map cell vertex ids to face vertex ids.
            for (size_t i = 0; i < 2; ++i)
            {
                for (size_t j = 0; j < 4; ++j)
                {
                    if (edge3D.getVertexIds()[i] == cell.getVertexIds()[j])
                    {
                        if (i == 0)
                        {
                            baryOut[j] = bary;
                        }
                        else
                        {
                            baryOut[j] = 1 - bary;
                        }
                        break;
                    }
                }
            }

            return true;
        }

        std::cout << "cell ids empty\n";
    }
    else if (poly->getDimensionType() == Polygon::DimensionType::TWO_D)
    {
        if (interpolator)
        {
            ID cellId;
            Polygon2DTopology* topo = static_cast<Polygon2DTopology*>(&poly->getTopology());
            TopologyFace& f = topo->getFace(e.getFaceIds()[0]);
            Eigen::Vector3d bary2 = Eigen::Vector::Zero();
            for (size_t i = 0; i < 2; ++i)
            {
                for (size_t j = 0; j < 3; ++j)
                {
                    if (e.getVertexIds()[i] == f.getVertexIds()[j])
                    {
                        if (i == 0)
                        {
                            bary2[j] = bary;
                        }
                        else
                        {
                            bary2[j] = 1 - bary;
                        }
                        break;
                    }
                }
            }
            baryOut = interpolator->calculateBary3(f.getID(), bary2, cellId);
            elementIdOut = cellId;
            return true;
        }
    }

    return false;
}

bool TriangleCollider::fillBarycentricCoordinates(
        Polygon* poly,
        TopologyVertex& v,
        MeshInterpolatorFEM* interpolator,
        ID& elementIdOut,
        Eigen::Vector4d& baryOut)
{
    baryOut = Eigen::Vector4d::Zero();
    if (poly->getDimensionType() == Polygon::DimensionType::THREE_D)
    {
        Polygon3DTopology* topo = static_cast<Polygon3DTopology*>(&poly->getTopology());

        // obtain cell id
        // face is of outer topology2d
        // required cell is w.r.t. face of corresponding Topology3D
        TopologyVertex& vertex3D = topo->getVertex(topo->getOuterVertexIds()[v.getID()]);
        if (vertex3D.getCellIds().size() > 0)
        {
            ID cellId = vertex3D.getCellIds()[0];
            TopologyCell& cell = topo->getCells()[cellId];

            elementIdOut = cellId;

            // Map cell vertex ids to face vertex ids.
            for (size_t j = 0; j < 4; ++j)
            {
                if (vertex3D.getID() == cell.getVertexIds()[j])
                {
                    baryOut[j] = 1.0;
                    break;
                }
            }

            return true;
        }
        std::cout << "cell ids empty\n";
    }
    else if (poly->getDimensionType() == Polygon::DimensionType::TWO_D)
    {
        if (interpolator)
        {
            ID cellId;
            Polygon2DTopology* topo = static_cast<Polygon2DTopology*>(&poly->getTopology());
            TopologyFace& f = topo->getFace(v.getFaceIds()[0]);
            Eigen::Vector3d bary2 = Eigen::Vector::Zero();
            for (size_t j = 0; j < 4; ++j)
            {
                if (v.getID() == f.getVertexIds()[j])
                {
                    bary2[j] = 1.0;
                    break;
                }
            }
            baryOut = interpolator->calculateBary3(f.getID(), bary2, cellId);
            elementIdOut = cellId;
            return true;
        }
    }
    return false;
}
