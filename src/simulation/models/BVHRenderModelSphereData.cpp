#include "BVHRenderModelSphereData.h"

#include <scene/data/geometric/GeometricSphere.h>
#include <scene/data/geometric/Polygon.h>
#include <scene/data/geometric/Polygon2D.h>

#include <scene/model/PolygonRenderModel.h>

#include <simulation/collision_detection/broad/BVSphere.h>

BVHRenderModelSphereData::BVHRenderModelSphereData(
        RenderModelManager* renderModelManager,
        BVSphere* bvSphere,
        int level,
        double radius,
        int resolution,
        bool isLeaf)
    : BVHRenderModelData (level, isLeaf)
    , mBvSphere(bvSphere)
    , mInitialized(false)
{
    mGeometricSphere = std::make_shared<GeometricSphere>(radius, resolution);

    mRenderModel = std::make_shared<PolygonRenderModel>(
                renderModelManager,
                mGeometricSphere->getPolygon());
}

BVHRenderModelSphereData::BVHRenderModelSphereData(
        RenderModelManager* renderModelManager,
        GeometricSphere* geometricSphereTemplate,
        BVSphere* bvSphere,
        int level,
        double radius,
        bool isLeaf)
    : BVHRenderModelData (isLeaf, level)
    , mBvSphere(bvSphere)
    , mInitialized(false)
{
    mRadiusPrevious = 0.0;

    if (geometricSphereTemplate)
    {
        mGeometricSphere = std::make_shared<GeometricSphere>(*geometricSphereTemplate);
        mRenderModel = std::make_shared<PolygonRenderModel>(
                    renderModelManager,
                    mGeometricSphere->getPolygon());
    }

    setRadius(radius);
}

BVHRenderModelSphereData::~BVHRenderModelSphereData()
{

}

void BVHRenderModelSphereData::initialize(
        Renderer* renderer,
        RenderModelManager* renderModelManager,
        GeometricSphere* geometricSphereTemplate)
{
    if (mInitialized)
        return;

    mInitialized = true;
    mGeometricSphere = std::make_shared<GeometricSphere>(*geometricSphereTemplate);
    mRenderModel = std::make_shared<PolygonRenderModel>(
                renderModelManager,
                mGeometricSphere->getPolygon());
    mRenderModel->addToRenderer(renderer);

    update();
}

void BVHRenderModelSphereData::setRadius(double radius)
{
    mRadius = radius;
    if (mGeometricSphere)
    {
        if (std::abs(mRadiusPrevious - mBvSphere->getRadius()) > 1e-5)
        {
            mGeometricSphere->setRadiusAndScale(radius);
            mGeometricSphere->getPolygon()->update();

            mRadiusPrevious = mBvSphere->getRadius();
        }
    }
}

void BVHRenderModelSphereData::setPosition(Eigen::Vector position)
{
    mGeometricSphere->getPolygon()->getTransform().translation() = position;
    mGeometricSphere->getPolygon()->update();
}

void BVHRenderModelSphereData::update()
{
    if (mRenderModel)
    {
        setRadius(mBvSphere->getRadius());
        setPosition(mBvSphere->getPosition());
        mRenderModel->update();
    }
}

void BVHRenderModelSphereData::addToRenderer(Renderer* renderer)
{
    if (mRenderModel)
        mRenderModel->addToRenderer(renderer);
}

void BVHRenderModelSphereData::removeFromRenderer(Renderer* renderer)
{
    if (mRenderModel)
        mRenderModel->removeFromRenderer(renderer);
}

void BVHRenderModelSphereData::setVisible(bool visible, int level)
{
    if (!mRenderModel)
        return;

    mRenderModel->setVisible(isToBeSetVisible(visible, level));
}

bool BVHRenderModelSphereData::isVisible() const
{
    if (!mRenderModel)
        return false;
    return mRenderModel->isVisible();
}

BoundingVolume::Type BVHRenderModelSphereData::getType() const
{
    return BoundingVolume::Type::SPHERE;
}
