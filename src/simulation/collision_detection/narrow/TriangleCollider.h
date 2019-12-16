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
class Polygon;
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
    TriangleCollider(double collisionMargin, bool invertNormalsIfNecessary = true);
    virtual ~TriangleCollider();

    void setCollisionMargin(double collisionMargin);
    void setInvertNormalsIfNecessary(bool invertNormals);

    void addTriangleSpherePair(CollisionTriangle& ct, CollisionSphere& cs);
    void addTrianglePair(CollisionTriangle& ct1, CollisionTriangle& ct2);
    void addSphereSpherePair(CollisionSphere& cs1, CollisionSphere& cs2);
    void addPair(
            Polygon2DTopology& topoSource,
            TopologyFeature& featureSource,
            SimulationObject* soSource,
            Polygon2DTopology& topoTarget,
            TopologyFeature& featureTarget,
            SimulationObject* soTarget);

    void clear();

    // Adds the detected collisions to the given vector.
    void collide(std::vector<Collision>& collisions);

    // Checks for a collision between the face and vertex. If there is one,
    // returns true and fills collision,
    bool collide(TopologyFace& f, TopologyVertex& v, Collision& collision);

    // Checks for a collision between the given edges. If there is one,
    // returns true and fills collision,
    bool collide(TopologyEdge& e1, TopologyEdge& e2, Collision& collision);

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

    SimulationObject* getSimulationObject(TopologyFeature* feature)
    {
        return mFeatureToSoMap[feature];
    }

    // Maps the given 3 barycentric coordinates bary of the face f to the
    // 4 barycentric coordinates of the element that the face is part of.
    // In baryOut at least 1 value will be zero.
    bool fillBarycentricCoordinates(
            Polygon* poly,
            TopologyFace& f,
            const Eigen::Vector& bary,
            ID& elementIdOut,
            std::array<double, 4>& baryOut);

    // Maps the given 2 barycentric coordinates bary of the edge e to the
    // 4 barycentric coordinates of the element that the edge is part of.
    // In baryOut at least 2 values will be zero.
    bool fillBarycentricCoordinates(
            Polygon* poly,
            TopologyEdge& e,
            double bary,
            ID& elementIdOut,
            std::array<double, 4>& baryOut);

    // Maps the given 1 barycentric coordinates bary of the vertex v to the
    // 4 barycentric coordinates of the element that the vertex is part of.
    // In baryOut 3 values are zero and 1 is one.
    bool fillBarycentricCoordinates(
            Polygon* poly,
            TopologyVertex& v,
            ID& elementIdOut,
            std::array<double, 4>& baryOut);

    typedef std::pair<TopologyFace*, TopologyVertex*> FVPair;
    typedef std::pair<TopologyEdge*, TopologyEdge*> EEPair;

    // The features pairs that potentially could collide.
    std::unordered_set<FVPair, boost::hash<FVPair>> mFeaturePairsFV;
    std::unordered_set<EEPair, boost::hash<EEPair>> mFeaturePairsEE;

    std::unordered_map<TopologyFeature*, SimulationObject*> mFeatureToSoMap;

    // Used as temporary memory to avoid unnecessary memory allocations
    IteratorPair mSourceTemp;
    IteratorPair mTargetTemp;

    double mMargin;
    double mMarginSquared;
    bool mInvertNormalsIfNecessary;

};

#endif // TRIANGLECOLLIDER_H
