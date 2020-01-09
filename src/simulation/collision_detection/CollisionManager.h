#ifndef COLLISIONMANAGER_H
#define COLLISIONMANAGER_H

#include <map>
#include <memory>
#include <proxy/ProxyDefs.h>
#include <scene/data/geometric/TopologyFace.h>
#include <simulation/collision_detection/narrow/Collider.h>
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

    bool collideAll();

    void updateAll();

    std::shared_ptr<Collider> getCollider();

    void setInvertNormalsIfNecessary(bool invertNormalsIfNecessary);
    bool getInvertNormalsIfNecessary() const;

    size_t getNumberOfBvhs() const;

    std::shared_ptr<BoundingVolumeHierarchy> getBoundingVolumeHierarchy(size_t index);

    std::shared_ptr<BoundingVolumeHierarchy> getBoundingVolumeHierarchy(SimulationObject* so);

    bool addListener(CollisionManagerListener* listener);
    bool removeListener(CollisionManagerListener* listener);

    void setCollisionMargin(double collisionMargin);
    double getCollisionMargin() const;

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
    };

    void addSimulationObject(
            const std::shared_ptr<SimulationObject>& so,
            const std::shared_ptr<Polygon>& polygon,
            const std::vector<std::shared_ptr<CollisionObject>>& collisionObjects,
            const std::shared_ptr<MeshInterpolatorFEM>& interpolator);

    std::map<unsigned int, double> calculateMinimumDistances(
            const std::vector<TopologyFace>& faces,
            const Vectors& positions);

    Domain* mDomain;

    std::shared_ptr<Collider> mCollider;

    std::vector<CollisionData> mCollisionData;

    std::vector<CollisionManagerListener*> mListeners;

    // If true, the next hierarchy update is forced for all objects.
    bool mForceUpdate;
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
            PROXY_FUNCTION(CollisionManager, mCm, setInvertNormalsIfNecessary,
                           PL(bool invertNormalsIfNecessary), PL(invertNormalsIfNecessary))
            )

#endif // COLLISIONMANAGER_H
