#ifndef COLLISIONMANAGER_H
#define COLLISIONMANAGER_H

#include <map>
#include <memory>
#include <proxy/ProxyDefs.h>
#include <simulation/collision_detection/narrow/Collider.h>
#include <vector>

class BoundingVolumeHierarchy;
class CollisionManagerListener;
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

    void addSimulationObject(
            std::shared_ptr<SimulationObject> so,
            std::shared_ptr<Polygon> polygon);

    bool removeSimulationObject(const std::shared_ptr<SimulationObject>& so);

    bool collideAll();

    void updateAll();

    std::shared_ptr<Collider> getCollider();

    size_t getNumberOfBvhs() const;

    std::shared_ptr<BoundingVolumeHierarchy> getBoundingVolumeHierarchy(size_t index);

    std::shared_ptr<BoundingVolumeHierarchy> getBoundingVolumeHierarchy(SimulationObject* so);

    bool addListener(CollisionManagerListener* listener);
    bool removeListener(CollisionManagerListener* listener);

private:
    struct CollisionData
    {
        std::shared_ptr<SimulationObject> mSo;
        std::shared_ptr<Polygon> mPolygon;
        std::shared_ptr<BoundingVolumeHierarchy> mBvh;
    };

    std::map<unsigned int, double> calculateMinimumDistances(
            const Faces& faces,
            const Vectors& positions);

    Domain* mDomain;

    std::shared_ptr<Collider> mCollider;

    std::vector<CollisionData> mCollisionData;

    std::vector<CollisionManagerListener*> mListeners;
};

PROXY_CLASS(CollisionManagerProxy, CollisionManager, mCm,
            PROXY_FUNCTION(CollisionManager, mCm, addSimulationObject,
                           PL(std::shared_ptr<SimulationObject> so,
                              std::shared_ptr<Polygon> polygon),
                           PL(so, polygon))
            PROXY_FUNCTION(CollisionManager, mCm, removeSimulationObject,
                           PL(const std::shared_ptr<SimulationObject>& so), PL(so))
            )

#endif // COLLISIONMANAGER_H
