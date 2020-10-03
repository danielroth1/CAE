#ifndef COLLISIONCONTROL_H
#define COLLISIONCONTROL_H

#include "CollisionManagerListener.h"

#include <map>
#include <memory>

class BoundingVolumeHierarchy;
class BVHRenderModel;
class ColliderRenderModel;
class CollisionManager;
class Domain;
class AbstractPolygon;
class RenderModelManager;
class SimulationObject;
class UIControl;


// CollisionControl a CollisionManager render components.
//
// Creates a CollisionManager that can be retrieved with
// getCollisionManager().
// Use the CollisionManager() to acces the collision detection.
//
// This class is responsible to access UI components that
// allow to visualize collision and collision hierarchies.
class CollisionControl : public CollisionManagerListener
{
public:

    // Create the CollisionManager and adds itself as a listener to update
    // ui elements according to changes in the CollisionManager.
    CollisionControl(Domain* domain,
                     UIControl* uiControl,
                     RenderModelManager* renderModelManager);

    ~CollisionControl();

    std::shared_ptr<CollisionManager> getCollisionManager();

    // Rendering methods
        void setBvhRenderLevel(int level);
        void setBvhRenderingEnables(bool visible);

        bool isCollisionsRenderingVisible() const;
        void setCollisionsRenderingVisible(bool visible);

    // CollisionManagerListener interface
public:
    virtual void notifySimulationObjectAdded(const std::shared_ptr<SimulationObject>& so) override;
    virtual void notifySimulationObjectRemoved(const std::shared_ptr<SimulationObject>& so) override;
    virtual void notifyCollideAllCalled() override;
    virtual void notifyUpdateAllCalled() override;

private:
    struct BVHData
    {
        BVHData()
        {
        }

        BVHData(std::shared_ptr<BoundingVolumeHierarchy> bvh,
                std::shared_ptr<BVHRenderModel> renderModel)
            : mBvh(bvh)
            , mRenderModel(renderModel)
        {
        }

        std::shared_ptr<BoundingVolumeHierarchy> mBvh;
        std::shared_ptr<BVHRenderModel> mRenderModel;
    };

    UIControl* mUiControl;
    RenderModelManager* mRenderModelManager;

    std::shared_ptr<CollisionManager> mCollisionManager;
    std::shared_ptr<ColliderRenderModel> mColliderRenderModel;

    std::map<std::shared_ptr<SimulationObject>, BVHData> mBvhMap;

    int mBvhRenderLevel;
    bool mBvhRenderingVisible;
    bool mCollisionsRenderingVisible;

};

#endif // COLLISIONCONTROL_H
