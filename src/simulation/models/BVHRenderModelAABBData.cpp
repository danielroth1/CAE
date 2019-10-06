#include "BVHRenderModelAABBData.h"

#include <RenderModelManager.h>

#include <simulation/collision_detection/broad/BVAABB.h>

#include <rendering/Renderer.h>

#include <scene/data/geometric/GeometricDataFactory.h>
#include <scene/data/geometric/Polygon2D.h>

#include <scene/model/PolygonRenderModel.h>


BVHRenderModelAABBData::BVHRenderModelAABBData(
        RenderModelManager* /*renderModelManager*/,
        BVAABB* aabb,
        int level,
        bool isLeaf)
    : BVHRenderModelData (level, isLeaf)
    , mAabb(aabb)
    , mInitialized(false)
{
}

void BVHRenderModelAABBData::initialize(Renderer* renderer,
                                        RenderModelManager* renderModelManager,
                                        const std::shared_ptr<Polygon2D>& boxTemplate)
{
    if (mInitialized)
        return;

    mBox = std::make_shared<Polygon2D>(*boxTemplate);
    mRenderModel = std::make_shared<PolygonRenderModel>(
                renderModelManager, mBox);
    mRenderModel->setWireframeEnabled(true);
    mRenderModel->addToRenderer(renderer);

    mInitialized = true;
}

void BVHRenderModelAABBData::update()
{
    if (mRenderModel)
    {
        const BoundingBox& bb = mAabb->getBoundingBox();
        Eigen::Vector3d scaling = bb.max() - bb.min();
        mBox->setTransform(
                    Eigen::Translation3d(0.5 * (bb.max() + bb.min())) *
                    Eigen::Scaling(scaling(0), scaling(1), scaling(2)));
        mBox->update();
        mRenderModel->update();
    }
}

void BVHRenderModelAABBData::addToRenderer(Renderer* renderer)
{
    if (mRenderModel)
    {
        mRenderModel->addToRenderer(renderer);
    }
}

void BVHRenderModelAABBData::removeFromRenderer(Renderer* renderer)
{
    if (mRenderModel)
    {
        mRenderModel->removeFromRenderer(renderer);
    }
}

void BVHRenderModelAABBData::setVisible(bool visible, int level)
{
    if (mRenderModel)
    {
        mRenderModel->setVisible(isToBeSetVisible(visible, level));
    }
}

bool BVHRenderModelAABBData::isVisible() const
{
    if (mRenderModel)
    {
        return mRenderModel->isVisible();
    }
    return false;
}

BoundingVolume::Type BVHRenderModelAABBData::getType() const
{
    return BoundingVolume::Type::AABB;
}
