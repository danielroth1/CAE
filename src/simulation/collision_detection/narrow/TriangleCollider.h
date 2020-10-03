#ifndef TRIANGLECOLLIDER_H
#define TRIANGLECOLLIDER_H

#include <boost/functional/hash.hpp>
#include <data_structures/DataStructures.h>
#include <map>
#include <scene/data/geometric/TopologyFeatureIterator.h>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>

class Collision;
class CollisionSphere;
class CollisionTriangle;
class MeshInterpolatorFEM;
class AbstractPolygon;
class Polygon2DTopology;
class SimulationObject;
class TopologyFace;
class TopologyFeature;
class TopologyVertex;
class TopologyEdge;

// Perfoms triangle based collision detection.
// Fills a data structure that stores all feature pairs that are close enough
// for collision detection (checked by BVH).
// Makes sure that no collision is duplicated by storing each feature pair
// only once.
// There won't by duplicated collisions from shared edges.
//
// Calculates for rigid the global collision point and for fem objects the
// element id and barycentric coordinates of the collision point.
class TriangleCollider
{
public:

    typedef std::tuple<SimulationObject*, ID, SimulationObject*, ID> CollisionTuple;
    typedef std::unordered_set<CollisionTuple, boost::hash<CollisionTuple>> CollisionTupleSet;

    TriangleCollider(double collisionMargin, bool invertNormalsIfNecessary = true);
    virtual ~TriangleCollider();

    void setCollisionMargin(double collisionMargin);
    void setInvertNormalsIfNecessary(bool invertNormals);

    void addTriangleSpherePair(CollisionTriangle& ct, CollisionSphere& cs);

    // Add all possible colliding feature pairs of the given collision triangles
    // to mFeaturePairsFV, mFeaturePairsVF, and mFeaturePairsEE. Performs
    // AABB checks for each feature pair combination:
    // For vertex - face: reuses the face AABB
    // For edge - edge: calculate (if not already done) the AABB for each edge
    //      and use those.
    // Only create feature pairs of features that are owned by the collision
    // triangle.
    void addTrianglePair(
            CollisionTriangle& ct1,
            CollisionTriangle& ct2,
            const CollisionTupleSet& ignoreFeatures);

    void addSphereSpherePair(CollisionSphere& cs1, CollisionSphere& cs2);

    // Slow way of adding a pair because it checks all possible feature combinations
    // and uses a slow unordered_set look up.
    void addPair(
            Polygon2DTopology& topoSource,
            TopologyFeature& featureSource,
            Polygon2DTopology& topoTarget,
            TopologyFeature& featureTarget);

    // Prepare new search operation between the two given polygons.
    void prepare(
            AbstractPolygon* poly1, AbstractPolygon* poly2,
            SimulationObject* so1, SimulationObject* so2,
            MeshInterpolatorFEM* interpolator1, MeshInterpolatorFEM* interpolator2,
            int runId);

    void clear();

    // Adds the detected collisions to the given vector.
    void collide(std::vector<Collision>& collisions);

    // Checks for a collision between the face and vertex. If there is one,
    // returns true and fills collision.
    // \param revertedFeatures - true if the feature f belongs to poly2 and
    //      v belongs to poly1
    bool collide(TopologyVertex& v,
                 TopologyFace& f,
                 bool revertedFeatures,
                 Collision& collision);

    // Checks for a collision between the given edges. If there is one,
    // returns true and fills collision.
    bool collide(TopologyEdge& e1,
                 TopologyEdge& e2,
                 Collision& collision);

    // Checks if the distance between the vertex and the face is within the
    // collision margin and if so, creates a collision and stores it in
    // the given collision parameter.
    bool collide(TopologyVertex& v,
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
    bool collide(TopologyEdge& e1,
                 TopologyEdge& e2,
                 SimulationObject* so1,
                 SimulationObject* so2,
                 MeshInterpolatorFEM* interpolator1,
                 MeshInterpolatorFEM* interpolator2,
                 AbstractPolygon* poly1,
                 AbstractPolygon* poly2,
                 Collision& collision);

private:

    struct FacePair
    {
        SimulationObject* so;
        TopologyFace& face;
    };

    struct IteratorPair
    {
        IteratorPair()
        {
            mFaceIteratorTemp = nullptr;
            mVertexIteratorTemp = nullptr;
        }

        ~IteratorPair()
        {
            if (mFaceIteratorTemp)
                delete mFaceIteratorTemp;

            if (mVertexIteratorTemp)
                delete mVertexIteratorTemp;
        }

        TopologyFeatureIterator* mFaceIteratorTemp;
        TopologyFeatureIterator* mVertexIteratorTemp;
    };

    // Returns a pointer to an iteratator that iterates over all features
    // of the given feature, so e.g. if feature is a face, then it iterates
    // over the face, its edges, and its vertices.
    //
    // Note: feature must be either a face or a vertex
    //
    // \param topp - the topology that the given feature is part of.
    // \param feature - the given feature
    // \param temp - temporary memory for the returned pointer. Either use
    //      mSourceTemp or mTargetTemp. The returned iterator can used for as
    //      long as this method isn't called another time with the same
    //      temporary variable.
    TopologyFeatureIterator* getFeatures(
            Polygon2DTopology& topo,
            TopologyFeature& feature,
            IteratorPair& temp);

    // Maps the given 3 barycentric coordinates bary of the face f to the
    // 4 barycentric coordinates of the element that the face is part of.
    // In baryOut at least 1 value will be zero.
    bool fillBarycentricCoordinates(
            AbstractPolygon* poly,
            TopologyFace& f,
            const Eigen::Vector& bary,
            MeshInterpolatorFEM* interpolator,
            ID& elementIdOut,
            Eigen::Vector4d& baryOut);

    // Maps the given 2 barycentric coordinates bary of the edge e to the
    // 4 barycentric coordinates of the element that the edge is part of.
    // In baryOut at least 2 values will be zero.
    bool fillBarycentricCoordinates(
            AbstractPolygon* poly,
            TopologyEdge& e,
            double bary,
            MeshInterpolatorFEM* interpolator,
            ID& elementIdOut,
            Eigen::Vector4d& baryOut);

    // Maps the given 1 barycentric coordinates bary of the vertex v to the
    // 4 barycentric coordinates of the element that the vertex is part of.
    // In baryOut 3 values are zero and 1 is one.
    bool fillBarycentricCoordinates(
            AbstractPolygon* poly,
            TopologyVertex& v,
            MeshInterpolatorFEM* interpolator,
            ID& elementIdOut,
            Eigen::Vector4d& baryOut);

    typedef std::pair<TopologyVertex*, TopologyFace*> VFPair;
    typedef std::pair<TopologyFace*, TopologyVertex*> FVPair;
    typedef std::pair<TopologyEdge*, TopologyEdge*> EEPair;

    // The features pairs that potentially could collide. Sets are used
    // to avoid duplicates.
    std::vector<FVPair> mFeaturePairsFV;
    std::vector<VFPair> mFeaturePairsVF;
    std::vector<EEPair> mFeaturePairsEE;
    std::unordered_set<FVPair, boost::hash<FVPair>> mFeaturePairsFVSet;
    std::unordered_set<EEPair, boost::hash<EEPair>> mFeaturePairsEESet;

    // Used as temporary memory to avoid unnecessary memory allocations
    IteratorPair mSourceTemp;
    IteratorPair mTargetTemp;

    double mMargin;
    double mMarginSquared;
    bool mInvertNormalsIfNecessary;

    // Simulation and geometric data of the two colliding objects.
    SimulationObject* mSo1;
    SimulationObject* mSo2;
    AbstractPolygon* mPoly1;
    AbstractPolygon* mPoly2;
    MeshInterpolatorFEM* mInterpolator1;
    MeshInterpolatorFEM* mInterpolator2;

    int mRunId;

    int edgeFails;
    int vertexFails;
    int eeFeaturePairs;
    int fvFeaturePairs;
};

#endif // TRIANGLECOLLIDER_H
