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
#include <simulation/collision_detection/broad/BoundingVolume.h>


TriangleCollider::TriangleCollider(
        double collisionMargin,
        bool invertNormalsIfNecessary)
    : mInvertNormalsIfNecessary(invertNormalsIfNecessary)
{
    mMargin = collisionMargin;
    mMarginSquared = collisionMargin * collisionMargin;
    mRunId = 0;
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
        CollisionTriangle& ct1,
        CollisionTriangle& ct2,
        const CollisionTupleSet& ignoreFeatures)
{
    // source
    Polygon2DTopology& topoSource = ct1.getAccessor()->getTopology2D();
    TopologyFace& face1 = topoSource.getFace(ct1.getFaceId());

    // target
    Polygon2DTopology& topoTarget = ct2.getAccessor()->getTopology2D();
    TopologyFace& face2 = topoTarget.getFace(ct2.getFaceId());

    assert(ct1.getAccessor()->getPolygon() == mPoly1 && ct2.getAccessor()->getPolygon()== mPoly2);

    // Update the edge AABBs for all not already visited faces. If a face
    // wasn't visited, mark it as visited by setting its run id to the one
    // of the current run.

#ifndef USE_EDGE_BV_STRATEGY_NONE
#ifndef USE_EDGE_BV_STRATEGY_ORIENTED_AABB
    if (ct1.getRunId() != mRunId)
    {
        ct1.setRunId(mRunId);
        ct1.updateEdgeBoundingBoxes(0.5 * mMargin);
    }
#endif
    if (ct2.getRunId() != mRunId)
    {
        ct2.setRunId(mRunId);
        ct2.updateEdgeBoundingBoxes(0.5 * mMargin);
    }
#endif

    // Slower version
//    addPair(topoSource, face1, topoTarget, face2);

    // Find all possible feature combinations of (vertex, face), (face, vertex),
    // and (edge, edge) between face1 and face2 using Representative-Triangles
    // (see class documentation of PolygonTopology for more infos).
    for (size_t i = 0; i < 3; ++i)
    {
        // Vertex -> Face
        if (face1.isVertexOwner(i))
        {
            unsigned int vId = face1.getVertexIds()[i];

            // Vertex-Face AABB check
            // It is necessary to add half the collision marign as offset because
            // other bounding volumes only add half the margin themselfs. Since
            // a vertex alone has no bounding volume, it's half margin must be
            // added here.
            if (ct2.getBoundingVolume()->isInside(ct1.getAccessor()->getPosition(vId),
                                                  0.5 * mMargin))
            {
                if (ignoreFeatures.find(std::make_tuple(
                                              mSo1, topoSource.getVertices()[vId].getGeometryID(),
                                              mSo2, face2.getGeometryID())) == ignoreFeatures.end())
                {
                    mFeaturePairsVF.push_back(VFPair(&topoSource.getVertices()[vId], &face2));
                }
            }
        }

        // Face -> Vertex
        if (face2.isVertexOwner(i))
        {
            unsigned int vId = face2.getVertexIds()[i];

            // Vertex-Face AABB check
            if (ct1.getBoundingVolume()->isInside(ct2.getAccessor()->getPosition(vId),
                                                  0.5 * mMargin))
            {
                if (ignoreFeatures.find(std::make_tuple(
                                              mSo1, face1.getGeometryID(),
                                              mSo2, topoTarget.getVertices()[vId].getGeometryID())) == ignoreFeatures.end())
                {
                    mFeaturePairsFV.push_back(FVPair(&face1, &topoTarget.getVertices()[vId]));
                }
            }
        }

        // Edge -> Edge
        unsigned int eId1 = face1.getEdgeIds()[i];
#ifdef USE_EDGE_BV_STRATEGY_ORIENTED_AABB
        const std::shared_ptr<Polygon2DAccessor>& accessor = ct1.getAccessor();
#endif
        if (face1.isEdgeOwner(i))
        {
#ifdef USE_EDGE_BV_STRATEGY_ORIENTED_AABB
            const Eigen::Vector3d& p1 = accessor->getPosition(
                        topoSource.getEdge(eId1).getVertexIds()[0]);
            const Eigen::Vector3d& p2 = accessor->getPosition(
                        topoSource.getEdge(eId1).getVertexIds()[1]);
#endif
            for (size_t j = 0; j < 3; ++j)
            {
                if (face2.isEdgeOwner(j))
                {
                    // Edge AABB check
#if defined(USE_EDGE_BV_STRATEGY_AABB) or defined(USE_EDGE_BV_STRATEGY_OBB)
                    if (ct1.getEdgeBoundingBoxes()[i].intersects(ct2.getEdgeBoundingBoxes()[j]))
#elif USE_EDGE_BV_STRATEGY_ORIENTED_AABB
                    if (ct2.getEdgeBoundingBoxes()[j].intersects(p1, p2))
#endif
                    {
                        unsigned int eId2 = face2.getEdgeIds()[j];
                        if (ignoreFeatures.find(
                                    std::make_tuple(
                                        mSo1, topoSource.getEdges()[eId1].getGeometryID(),
                                        mSo2, topoTarget.getEdges()[eId2].getGeometryID())) == ignoreFeatures.end())
                        {
                            mFeaturePairsEE.push_back(EEPair(&topoSource.getEdges()[eId1],
                                                             &topoTarget.getEdges()[eId2]));
                        }
                    }
                }
            }
        }
    }
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
        MeshInterpolatorFEM* interpolator1, MeshInterpolatorFEM* interpolator2,
        int runId)
{
    mPoly1 = poly1;
    mPoly2 = poly2;
    mSo1 = so1;
    mSo2 = so2;
    mInterpolator1 = interpolator1;
    mInterpolator2 = interpolator2;

    if (runId != mRunId)
    {
        // prints #of collisions / #of positive AABB checks
//        std::cout << "fails = " << edgeFails << " / " << eeFeaturePairs
//                  << ", " << vertexFails << " / " << fvFeaturePairs << "\n";
        edgeFails = 0;
        vertexFails = 0;
        eeFeaturePairs = 0;
        fvFeaturePairs = 0;
    }
    mRunId = runId;

    clear();

//    std::cout << "feature pairs = " << eeFeaturePairs << ", " << fvFeaturePairs << "\n";
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

    size_t numTotalPairs = mFeaturePairsVF.size() +
            mFeaturePairsFV.size() +
            mFeaturePairsEE.size();

    eeFeaturePairs += mFeaturePairsEE.size();
    fvFeaturePairs += mFeaturePairsFV.size();
    fvFeaturePairs += mFeaturePairsVF.size();

    if (numTotalPairs == 0)
        return;

    bool parallel = numTotalPairs > 8 * 100;

//    if (numTotalPairs > 8 * 40)
//        std::cout << "numTotalPairs = " << numTotalPairs << "\n";
//    if (numTotalPairs > 0)
//        std::cout << "sizes = " << mFeaturePairsVF.size() << ", " << mFeaturePairsFV.size() << ", " << mFeaturePairsEE.size() << "\n";
//    if (parallel)
//        std::cout << "numTotalPairs = " << numTotalPairs << "\n";

    if (!parallel)
    {
        // Vertex-Face
        for (size_t i = 0; i < mFeaturePairsVF.size(); ++i)
        {
            const std::pair<TopologyVertex*, TopologyFace*>& pair = mFeaturePairsVF[i];
            Collision c;
            bool collides = collide(*pair.first, *pair.second, false, c);
            if (collides)
            {
                collisions.push_back(c);
            }
            else
                ++vertexFails;
        }

        // Face-Vertex
        for (size_t i = 0; i < mFeaturePairsFV.size(); ++i)
        {
            const std::pair<TopologyFace*, TopologyVertex*>& pair = mFeaturePairsFV[i];
            Collision c;
            bool collides = collide(*pair.second, *pair.first, true, c);
            if (collides)
            {
                collisions.push_back(c);
            }
            else
                ++vertexFails;
        }

        // Edge-Edge
        for (size_t i = 0; i < mFeaturePairsEE.size(); ++i)
        {
            const std::pair<TopologyEdge*, TopologyEdge*>& pair = mFeaturePairsEE[i];
            Collision c;
            bool collides = collide(*pair.first, *pair.second, c);
            if (collides)
            {
                collisions.push_back(c);
            }
            else
                ++edgeFails;
        }
    }
    else
    {
        int nThreads = 8; //omp_get_num_threads() always returns 1.
        size_t pairsPerThread = std::ceil(static_cast<double>(numTotalPairs) / nThreads);
        std::vector<std::vector<Collision>> threadsCollisions;
        threadsCollisions.resize(nThreads);

#pragma omp parallel for
        for (int tId = 0; tId < nThreads; ++tId)
        {
            std::vector<Collision>& threadCollisions = threadsCollisions[tId];
            size_t start = tId * pairsPerThread;
            size_t end = std::min(numTotalPairs, (tId + 1) * pairsPerThread);

            for (size_t i = start; i < end; ++i)
            {
                if (i < mFeaturePairsVF.size())
                {
                    const std::pair<TopologyVertex*, TopologyFace*>& pair = mFeaturePairsVF[i];
                    Collision c;
                    bool collides = collide(*pair.first, *pair.second, false, c);
                    if (collides)
                    {
                        threadCollisions.push_back(c);
                    }
                }
                else if (i < mFeaturePairsVF.size() + mFeaturePairsFV.size())
                {
                    size_t index = i - mFeaturePairsVF.size();
                    const std::pair<TopologyFace*, TopologyVertex*>& pair = mFeaturePairsFV[index];
                    Collision c;
                    bool collides = collide(*pair.second, *pair.first, true, c);
                    if (collides)
                    {
                        threadCollisions.push_back(c);
                    }
                }
                else
                {
                    size_t index = i - mFeaturePairsVF.size() - mFeaturePairsFV.size();
                    const std::pair<TopologyEdge*, TopologyEdge*>& pair = mFeaturePairsEE[index];
                    Collision c;
                    bool collides = collide(*pair.first, *pair.second, c);
                    if (collides)
                    {
                        threadCollisions.push_back(c);
                    }
                }
            }
        }

        for (int i = 0; i < nThreads; ++i)
        {
            std::vector<Collision>& threadCollisions = threadsCollisions[i];
            for (size_t j = 0; j < threadCollisions.size(); ++j)
            {
                collisions.push_back(threadCollisions[j]);
            }
        }
    }
}

bool TriangleCollider::collide(
        TopologyVertex& v,
        TopologyFace& f,
        bool reverted,
        Collision& collision)
{
    if (!reverted)
        return collide(v, f, mSo1, mSo2, mInterpolator1, mInterpolator2, mPoly1, mPoly2, reverted, collision);
    else
        return collide(v, f, mSo2, mSo1, mInterpolator2, mInterpolator1, mPoly2, mPoly1, reverted, collision);
}

bool TriangleCollider::collide(
        TopologyEdge& e1,
        TopologyEdge& e2,
        Collision& collision)
{
    return collide(e1, e2, mSo1, mSo2, mInterpolator1, mInterpolator2, mPoly1, mPoly2, collision);
}

bool TriangleCollider::collide(
        TopologyVertex& v, TopologyFace& f,
        SimulationObject* so1, SimulationObject* so2,
        MeshInterpolatorFEM* interpolator1, MeshInterpolatorFEM* interpolator2,
        Polygon* poly1, Polygon* poly2,
        bool revertedFeaturePair,
        Collision& collision)
{
    // so1 / poly1 / interpolator1 contain v
    // so2 / poly2 / interpolator2 contain f

    // first object (so1, poly1, interpolator1) -> vertex
    // second object (so2, poly2, interpolator2) -> face

    // Collisions between non-polygons aren't supported
    if (so1->getGeometricData()->getType() != GeometricData::Type::POLYGON ||
        so2->getGeometricData()->getType() != GeometricData::Type::POLYGON)
    {
        return false;
    }

    Eigen::Vector& pos = poly1->getAccessor2D()->getPosition(v.getID());

    Eigen::Vector inter; // projected point
    Eigen::Vector bary; // baryzentric coordinates
    bool projectsOnTriangle;

    Eigen::Vector& p1 = poly2->getPosition(f.getVertexIds()[0]);
    Eigen::Vector& p2 = poly2->getPosition(f.getVertexIds()[1]);
    Eigen::Vector& p3 = poly2->getPosition(f.getVertexIds()[2]);

    bool ok = MathUtils::projectPointOnTriangle(
                p1, p2, p3,
                pos,
                inter, bary, projectsOnTriangle);

    // Ignore vertex-face collisions that don't project on the triangle.
    // There are situations where needed collisions are not found but most
    // of the times redundant collisions that are already found in the
    // edge-edge case are avoided.
    if (ok /*&& projectsInTriangle */&& (pos - inter).squaredNorm() < mMarginSquared)
    {
        // Determin collsion normal -> triangle normal...
        Eigen::Vector dir;
        if (projectsOnTriangle)
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

        bool isInside =
                poly1->isInside(v, inter) &&
                poly2->isInside(f, pos);
        // We don't want to invert face normals because they always point out
        // of the object (thats why the !projectsOnTriangle is used).
        if (mInvertNormalsIfNecessary)
        {
            if (isInside && !projectsOnTriangle)
                dir = -dir;
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

        // Collisions store elements in the order in which they are checked
        // in the collision detection. The first polygon has always a lower
        // global id than the second one.
        if (!revertedFeaturePair)
        {
            new (&collision) Collision(&v, &f, interpolator1, interpolator2,
                                       so1, so2,
                                       pos, inter,
                                       dir, 0.0,
                                       v1Index,
                                       v2Index,
                                       isInside);

            ID elementId;
            bool ok = fillBarycentricCoordinates(poly1, v, interpolator1, elementId, collision.getBarycentricCoordiantesA());
            collision.setElementIdA(elementId);

            ok = ok && fillBarycentricCoordinates(poly2, f, bary, interpolator2, elementId, collision.getBarycentricCoordiantesB());
            collision.setElementIdB(elementId);

//            if (!ok)
//                return false;
        }
        else
        {
            // revert collision
            new (&collision) Collision(&f, &v, interpolator2, interpolator1,
                                       so2, so1,
                                       inter, pos,
                                       -dir, 0.0,
                                       v2Index,
                                       v1Index,
                                       isInside);

            ID elementId;
            bool ok = fillBarycentricCoordinates(poly1, v, interpolator1, elementId, collision.getBarycentricCoordiantesB());
            collision.setElementIdB(elementId);

            ok = ok && fillBarycentricCoordinates(poly2, f, bary, interpolator2, elementId, collision.getBarycentricCoordiantesA());
            collision.setElementIdA(elementId);

//            if (!ok)
//                return false;
        }

        return true;
    }

    return false;
}

bool TriangleCollider::collide(
        TopologyEdge& e1, TopologyEdge& e2,
        SimulationObject* so1, SimulationObject* so2,
        MeshInterpolatorFEM* interpolator1, MeshInterpolatorFEM* interpolator2,
        Polygon* poly1, Polygon* poly2,
        Collision& collision)
{
    // Collisions between non-polygons aren't supported
    if (so1->getGeometricData()->getType() != GeometricData::Type::POLYGON ||
        so2->getGeometricData()->getType() != GeometricData::Type::POLYGON)
    {
        return false;
    }

    const Eigen::Vector& x11 =
            poly1->getAccessor2D()->getPosition(e1.getVertexIds()[0]);
    const Eigen::Vector& x12 =
            poly1->getAccessor2D()->getPosition(e1.getVertexIds()[1]);

    const Eigen::Vector& x21 =
            poly2->getAccessor2D()->getPosition(e2.getVertexIds()[0]);
    const Eigen::Vector& x22 =
            poly2->getAccessor2D()->getPosition(e2.getVertexIds()[1]);

    Eigen::Vector3d inter1; // projected point 1
    Eigen::Vector3d inter2; // projected point 2
    Eigen::Vector2d bary; // baryzentric coordinates
    bool projectsOnEdges;

    bool ok = MathUtils::projectEdgeOnEdge(
                x11, x12, x21, x22, inter1, inter2, bary, projectsOnEdges);

    // Ignore points that don't project on both edges. These contacts are
    // usually already covered by the a vertex-face collision. In some
    // situations they currently are not, though.
    if (ok && projectsOnEdges && (inter1 - inter2).squaredNorm() < mMarginSquared)
    {
        Eigen::Vector dir = (inter1 - inter2).normalized();

        bool isInside =
                poly1->isInside(e1, inter2) &&
                poly2->isInside(e2, inter1);
        if (mInvertNormalsIfNecessary)
        {
            if (isInside)
                dir = -dir;
        }

        ID v1Index = 0;
        ID v2Index = 0;

        // TODO: probably not needed anymore because collisions are described
        // by either a vector from center to collision point (rigids) or
        // by barycentric coordinates (deformable)
//        if (poly1->getDimensionType() == Polygon::DimensionType::THREE_D)
//        {
//            Polygon3DTopology* t3 = static_cast<Polygon3DTopology*>(&poly1->getTopology());

//            if (bary(0) < 0.5)
//                v1Index = t3->getOuterEdges()[e1.getID()].getVertexIds()[0];
//            else
//                v1Index = t3->getOuterEdges()[e1.getID()].getVertexIds()[1];

//        }

//        if (poly2->getDimensionType() == Polygon::DimensionType::THREE_D)
//        {
//            Polygon3DTopology* t3 = static_cast<Polygon3DTopology*>(&poly2->getTopology());
//            if (bary(1) < 0.5)
//                v2Index = t3->getOuterEdges()[e2.getID()].getVertexIds()[0];
//            else
//                v2Index = t3->getOuterEdges()[e2.getID()].getVertexIds()[0];
//        }

        new (&collision) Collision(
                    &e1, &e2, interpolator1, interpolator2,
                    so1, so2,
                    inter1, inter2, dir, 0.0, v1Index, v2Index, isInside);

        ID elementId;
        bool ok = fillBarycentricCoordinates(poly1, e1, bary(0), interpolator1, elementId, collision.getBarycentricCoordiantesA());
        collision.setElementIdA(elementId);

        ok = ok && fillBarycentricCoordinates(poly2, e2, bary(1), interpolator2, elementId, collision.getBarycentricCoordiantesB());
        collision.setElementIdB(elementId);

//        if (!ok)
//            return false;

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
            bool ok;
            baryOut = interpolator->calculateBary3(f.getID(), bary, cellId, ok);
//            if (!ok)
//                return false;

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

            bool ok;
            baryOut = interpolator->calculateBary3(f.getID(), bary2, cellId, ok);
//            if (!ok)
//                return false;

            elementIdOut = cellId;
            return true;
        }
    }

    return true;
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

            bool ok;
            baryOut = interpolator->calculateBary3(f.getID(), bary2, cellId, ok);
//            if (!ok)
//                return false;

            elementIdOut = cellId;
            return true;
        }
    }
    return true;
}
