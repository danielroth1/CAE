#include "CollisionControl.h"
#include "CollisionManager.h"

#include <simulation/models/BVHRenderModel.h>
#include <simulation/models/ColliderRenderModel.h>

#include <ui/UIControl.h>

#include <rendering/Renderer.h>

CollisionControl::CollisionControl(
        Domain* domain,
        UIControl* uiControl,
        RenderModelManager* renderModelManager)
    : mUiControl(uiControl)
    , mRenderModelManager(renderModelManager)
{
    mCollisionManager = std::make_shared<CollisionManager>(domain);
    mCollisionManager->addListener(this);

    mColliderRenderModel = std::make_shared<ColliderRenderModel>(mCollisionManager->getCollider());
    mColliderRenderModel->addToRenderer(uiControl->getRenderer());
}

std::shared_ptr<CollisionManager> CollisionControl::getCollisionManager()
{
    return mCollisionManager;
}

void CollisionControl::setBvhRenderLevel(int level)
{
    mBvhRenderLevel = level;

    for (std::pair<std::shared_ptr<SimulationObject>, BVHData> p : mBvhMap)
    {
        BVHData& data = p.second;
        data.mRenderModel->setRenderedLevel(level);
    }
}

void CollisionControl::setBvhRenderingEnables(bool visible)
{
    for (std::pair<std::shared_ptr<SimulationObject>, BVHData> p : mBvhMap)
    {
        BVHData& data = p.second;
        data.mRenderModel->setVisible(visible);
    }
}

bool CollisionControl::isCollisionsRenderingVisible() const
{
    return mColliderRenderModel->isVisible();
}

void CollisionControl::setCollisionsRenderingVisible(bool visible)
{
    mColliderRenderModel->setVisible(visible);
}

void CollisionControl::notifySimulationObjectAdded(std::shared_ptr<SimulationObject>& so)
{
    // add BVHData
    std::shared_ptr<BoundingVolumeHierarchy> bvh =
            mCollisionManager->getBoundingVolumeHierarchy(so.get());

    std::shared_ptr<BVHRenderModel> renderModel =
            std::make_shared<BVHRenderModel>(mRenderModelManager, bvh);

    renderModel->setVisible(false);
    renderModel->addToRenderer(mUiControl->getRenderer());

    BVHData data(bvh, renderModel);

    mBvhMap[so] = data;
}

void CollisionControl::notifySimulationObjectRemoved(const std::shared_ptr<SimulationObject>& so)
{
    // remove BVHData
    auto it = mBvhMap.find(so);

    if (it != mBvhMap.end())
    {
        it->second.mRenderModel->removeFromRenderer(mUiControl->getRenderer());
        mBvhMap.erase(it);
    }
}

void CollisionControl::notifyCollideAllCalled()
{
    // update ColliderRenderModel
    mColliderRenderModel->update();
}

void CollisionControl::notifyUpdateAllCalled()
{
    // update all BoundingVolumeRenderModels
    for (std::pair<std::shared_ptr<SimulationObject>, BVHData> p : mBvhMap)
    {
        BVHData& data = p.second;
        data.mRenderModel->update();
    }
}
