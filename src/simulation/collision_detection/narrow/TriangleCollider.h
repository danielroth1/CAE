#ifndef TRIANGLECOLLIDER_H
#define TRIANGLECOLLIDER_H

#include <boost/functional/hash.hpp>
#include <map>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>

class Collision;
class CollisionSphere;
class CollisionTriangle;
class Polygon2DTopology;
class SimulationObject;
class TopologyVertex;
class TopologyEdge;
class TopologyFace;
class TopologyFeature;

// Perfoms triangle based collision detection.
// Fills a data structure that stores all feature pairs that are close enough
// for collision detection (checked by BVH).
// Makes sure that no collision is duplicated by storing each feature pair
// only once.
// There won't by duplicated collisions from shared edges.
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

    std::vector<TopologyFeature*> getFeatures(
            Polygon2DTopology& topo, TopologyFeature& feature);

    SimulationObject* getSimulationObject(TopologyFeature* feature)
    {
        return mFeatureToSoMap[feature];
    }

    typedef std::pair<TopologyFace*, TopologyVertex*> FVPair;
    typedef std::pair<TopologyEdge*, TopologyEdge*> EEPair;

    // The features pairs that potentially could collide.
    std::unordered_set<FVPair, boost::hash<FVPair>> mFeaturePairsFV;
    std::unordered_set<EEPair, boost::hash<EEPair>> mFeaturePairsEE;

    std::unordered_map<TopologyFeature*, SimulationObject*> mFeatureToSoMap;

    double mMargin;
    double mMarginSquared;
    bool mInvertNormalsIfNecessary;

};

#endif // TRIANGLECOLLIDER_H
