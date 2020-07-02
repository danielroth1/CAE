#ifndef COLLISIONMANAGER_H
#define COLLISIONMANAGER_H

#include <boost/functional/hash.hpp>
#include <proxy/ProxyDefs.h>
#include <scene/data/geometric/TopologyFace.h>
#include <simulation/SimulationCollision.h>
#include <simulation/collision_detection/narrow/Collider.h>

#include <map>
#include <memory>
#include <tuple>
#include <unordered_set>
#include <vector>

class BoundingVolumeHierarchy;
class CollisionManagerListener;
class MeshInterpolatorFEM;
class Polygon;
class SimulationObject;

enum class CollisionObjectType
{
    SPHERE, TRIANGLE
};

enum class BoundingVolumeType
{
    SPHERE, AABB
};

class CollisionManager : public std::enable_shared_from_this<CollisionManager>
{
public:

    typedef std::tuple<SimulationObject*, ID, SimulationObject*, ID> CollisionTuple;
    typedef std::unordered_set<CollisionTuple, boost::hash<CollisionTuple>> CollisionTupleSet;

    CollisionManager(Domain* domain);

    Domain* getDomain();

    void addSimulationObjectTriangles(
            const std::shared_ptr<SimulationObject>& so,
            const std::shared_ptr<Polygon>& polygon,
            const std::shared_ptr<MeshInterpolatorFEM>& interpolator = nullptr);

    // Adds a deformable object that has two different geometries:
    // - the one used for deformation simulation = interpolation source
    // - the one used for collision handling = interpolation target
    // The simulation objects polygon must be the same as the source polygon
    // of the given interpolation.
    void addSimulationObjectTriangles(
            const std::shared_ptr<SimulationObject>& so,
            const std::shared_ptr<MeshInterpolatorFEM>& interpolator);

    void addSimulationObject(
            std::shared_ptr<SimulationObject> so,
            std::shared_ptr<Polygon> polygon,
            double sphereDiameter = 0.1);

    bool removeSimulationObject(const std::shared_ptr<SimulationObject>& so);

    // Adds a collision group id to the given simulation object. If this
    // simulation object isn't collidable, nothing happens.
    // All objects that share a collision group id can't collide with each other.
    void addCollisionGroupId(
            const std::shared_ptr<SimulationObject>& so, int groupId);

    // Sets all collision group ids for the given simulation object. If this
    // simulation object isn't collidable, nothing happens.
    // All objects that share a collision group id can't collide with each other.
    void setCollisionGroupIds(
            const std::shared_ptr<SimulationObject>& so,
            const std::vector<int>& groupIds);

    // Checks for all pair of collidable simulation objects (those that were
    // added with addSimulationObject) if they are colliding.
    // Note: Doesn't use a bounding volume hierarchy to find close pairs
    // of simulation object. To check the triangle-triangle collisions,
    // a bounding volume is used again. This means that this method can
    // be inefficient if there are many collidable objects in the scene.
    bool collideAll();

    void updateAll();

    // Publishes all collidable geometric data. Does not notify listeners.
    // All collidable MeshInterpolators are published as well.
    void updateGeometries();

    // Revalidates the currently stored collisions. Does so by performing a
    // triangle collision detection on each of them and updating there values.
    // Collisions that aren't valid anymore (collision distance bigger than
    // threshold) are removed. Because this operation doesn't requires to
    // iterate any bounding volume hierarchies and is restricted to the
    // lasts step valid collisions, it is fairly cheap.
    void revalidateCollisions();

    std::shared_ptr<Collider> getCollider();
    const std::vector<SimulationCollision>& getCollisions() const;

    void setInvertNormalsIfNecessary(bool invertNormalsIfNecessary);
    bool getInvertNormalsIfNecessary() const;

    size_t getNumberOfBvhs() const;

    std::shared_ptr<BoundingVolumeHierarchy> getBoundingVolumeHierarchy(size_t index);

    std::shared_ptr<BoundingVolumeHierarchy> getBoundingVolumeHierarchy(SimulationObject* so);

    bool addListener(CollisionManagerListener* listener);
    bool removeListener(CollisionManagerListener* listener);

    void setCollisionMargin(double collisionMargin);
    double getCollisionMargin() const;

    size_t getNumContacts() const;

    static bool createEdges;

private:
    struct CollisionData
    {
        // To detect if the data changed before updating everything.
        Eigen::Vector3d mX;
        Eigen::Quaterniond mQ;

        std::shared_ptr<SimulationObject> mSo;
        std::shared_ptr<Polygon> mPolygon;
        std::shared_ptr<BoundingVolumeHierarchy> mBvh;

        std::vector<int> mCollisionGroups;
    };

    void addSimulationObject(
            const std::shared_ptr<SimulationObject>& so,
            const std::shared_ptr<Polygon>& polygon,
            const std::vector<std::shared_ptr<CollisionObject>>& collisionObjects,
            const std::shared_ptr<MeshInterpolatorFEM>& interpolator);

    std::map<unsigned int, double> calculateMinimumDistances(
            const std::vector<TopologyFace>& faces,
            const Vectors& positions);

    // Disclaimer: This method doesn't really work as expected but the idea
    // is probably right. Simply too many important inside collisions are removed.
    // Maybe there will be sth. in the future that improves the filtering out process.
    //
    // This method removes all collisions that are inside an object
    // (isInside == true) except for the closest numAllowed ones.
    // It removes collisions from mSimulationCollisions[offset ... mSimulationCollisions.size()].
    // The order of collisions remains the same.
    //
    // The idea is that in general isInside contacts can bring more harm than
    // good since they can easily contradict each other. Preferably, we want
    // constraints that try to reduce the intersection volume and this could
    // be a simple way to achieve this.
    //
    // \param offset - first number of collisions that are ignored. Filtering is
    //      only applied to collisions with indices starting from offset.
    // \param numAllowed - number of isInside collisions that are allowed (not
    //      counting the first offset ignored ones.
    void filterCollisions(size_t offset, size_t numAllowed = 1);

    // Returns true if the two given vectors share a number. Assumes them
    // to be sorted.
    bool isSharingCollisionGroups(
            const std::vector<int>& groupIdsA,
            const std::vector<int>& groupIdsB);

    Domain* mDomain;

    std::shared_ptr<Collider> mCollider;

    std::vector<CollisionData> mCollisionData;

    std::vector<CollisionManagerListener*> mListeners;

    std::vector<SimulationCollision> mSimulationCollisions;

    // SimualtionObject*, FeauterId A, SimulationObject*, FeatureID B
    // Stores the ids of feature pairs of collisions that are stored. Offers
    // a fast look up if a collision is already stored and can be ignored.
    CollisionTupleSet mAlreadySeenCollisions;

    // If true, the next hierarchy update is forced for all objects.
    bool mForceUpdate;

    int mRunId;

    // A contact is a collision that persists over multiple time steps. All
    // collisions that were valid for more than 1 step are considered contacts.
    size_t mNumContacts;
};

PROXY_CLASS(CollisionManagerProxy, CollisionManager, mCm,
            PROXY_FUNCTION(CollisionManager, mCm, addSimulationObjectTriangles,
                           PL(std::shared_ptr<SimulationObject> so,
                              std::shared_ptr<Polygon> polygon),
                           PL(so, polygon))
            PROXY_FUNCTION(CollisionManager, mCm, addSimulationObjectTriangles,
                           PL(std::shared_ptr<SimulationObject> so,
                              std::shared_ptr<MeshInterpolatorFEM> interpolation),
                           PL(so, interpolation))
            PROXY_FUNCTION(CollisionManager, mCm, addSimulationObject,
                           PL(std::shared_ptr<SimulationObject> so,
                              std::shared_ptr<Polygon> polygon,
                              double sphereDiameter),
                           PL(so, polygon, sphereDiameter))
            PROXY_FUNCTION(CollisionManager, mCm, removeSimulationObject,
                           PL(const std::shared_ptr<SimulationObject>& so), PL(so))
            PROXY_FUNCTION(CollisionManager, mCm, addCollisionGroupId,
                           PL(const std::shared_ptr<SimulationObject>& so,
                              int groupId),
                           PL(so, groupId))
            PROXY_FUNCTION(CollisionManager, mCm, setCollisionGroupIds,
                           PL(const std::shared_ptr<SimulationObject>& so,
                              const std::vector<int>& groupIds),
                           PL(so, groupIds))
            PROXY_FUNCTION(CollisionManager, mCm, setInvertNormalsIfNecessary,
                           PL(bool invertNormalsIfNecessary), PL(invertNormalsIfNecessary))
            )

#endif // COLLISIONMANAGER_H
