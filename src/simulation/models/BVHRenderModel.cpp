#include "BVHRenderModel.h"

#include <simulation/collision_detection/broad/BVHCore.h>
#include <simulation/collision_detection/broad/BVSphere.h>
#include <simulation/collision_detection/broad/BoundingVolumeHierarchy.h>

#include <scene/data/geometric/GeometricSphere.h>
#include <scene/data/geometric/Polygon.h>
#include <scene/data/geometric/Polygon2D.h>

#include <scene/model/PolygonRenderModel.h>

#include <rendering/Renderer.h>

#include <rendering/object/RenderPolygon2D.h>

#include <RenderModelManager.h>


BVHRenderModel::BVHRenderModel(
        RenderModelManager* renderModelManager,
        std::shared_ptr<BoundingVolumeHierarchy> bvh)
    : mRenderModelManager(renderModelManager)
    , mBvh(bvh)
{

    mRenderedLevel = 0;
    mSphereResolution = 2;

    mGeometricSphereTemplate = std::make_shared<GeometricSphere>(0.04, mSphereResolution);

    reset();
}

BVHRenderModel::~BVHRenderModel()
{
    for (std::vector<std::shared_ptr<SphereData>>& datas : mRenderPolygons)
    {
        datas.clear();
    }
}

void BVHRenderModel::reset()
{
    mRenderPolygons.clear();

    BVHTreeTraverser traverser(mBvh->getRoot());

    class InitRenderModelVisitor : public BVHNodeVisitor
    {
    public:
        InitRenderModelVisitor(BVHRenderModel& _rm,
                               BVHTreeTraverser& _traverser)
            : rm(_rm)
            , traverser(_traverser)
        {
        }

        virtual void visit(BVHChildrenNode* childrenNode)
        {
            if (childrenNode->getData())
            {
                size_t renderLevel = traverser.getCurrentLevel();

                // root has no bounding volume?
                // not initialized yet. Initialize data to nullptr and check for that here?
                rm.addSphere(childrenNode->getData()->getBoundingVolume(), renderLevel, false);
            }
        }

        virtual void visit(BVHLeafNode* leafNode)
        {
            size_t renderLevel = traverser.getCurrentLevel();

            rm.addSphere(leafNode->getData()->getBoundingVolume(), renderLevel, true);
        }

        BVHRenderModel& rm;
        BVHTreeTraverser& traverser;
    } visitor(*this, traverser);


    traverser.traverse(visitor);
}

void BVHRenderModel::update()
{
    for (const std::vector<std::shared_ptr<SphereData>>& levels : mRenderPolygons)
    {
        for (const std::shared_ptr<SphereData>& sphereData : levels)
        {
            sphereData->update();
        }
    }
}

void BVHRenderModel::revalidate()
{
    reset();
}

void BVHRenderModel::accept(RenderModelVisitor& /*v*/)
{

}

void BVHRenderModel::addToRenderer(Renderer* renderer)
{
    mRenderer = renderer;
    for (const std::vector<std::shared_ptr<SphereData>>& levels : mRenderPolygons)
    {
        for (const std::shared_ptr<SphereData>& sphereData : levels)
        {
            sphereData->addToRenderer(renderer);
        }
    }
}

void BVHRenderModel::removeFromRenderer(Renderer* renderer)
{
    for (const std::vector<std::shared_ptr<SphereData>>& levels : mRenderPolygons)
    {
        for (const std::shared_ptr<SphereData>& sphereData : levels)
        {
            sphereData->removeFromRenderer(renderer);
        }
    }
}

void BVHRenderModel::setVisible(bool visible)
{
    for (size_t i = 0; i < mRenderPolygons.size(); ++i)
    {
        const std::vector<std::shared_ptr<SphereData>>& levelData = mRenderPolygons[i];
        for (const std::shared_ptr<SphereData>& sphereData : levelData)
        {
            if (sphereData->isToBeSetVisible(visible, mRenderedLevel))
                sphereData->initialize(mRenderer,
                                       mRenderModelManager,
                                       mGeometricSphereTemplate.get());
            sphereData->setVisible(visible, mRenderedLevel);
        }
    }
    RenderModel::setVisible(visible);
}

void BVHRenderModel::setRenderedLevel(int renderLevel)
{
    mRenderedLevel = std::max(-2, std::min(static_cast<int>(mRenderPolygons.size()) - 1, renderLevel));

    std::cout << "set render level " << mRenderedLevel << "\n";
    // update visibility according to new render level
    setVisible(mVisible);
}

void BVHRenderModel::addSphere(BoundingVolume* bv, size_t level, bool isLeaf)
{
    class SphereDispatcher : public BoundingVolumeVisitor
    {
    public:
        SphereDispatcher(BVHRenderModel& _renderModel, size_t _level, bool _isLeaf)
            : renderModel(_renderModel)
            , level(_level)
            , isLeaf(_isLeaf)
        {

        }

        virtual void visit(BVSphere* sphere)
        {
            renderModel.addSphere(sphere, level, isLeaf);
        }

        BVHRenderModel& renderModel;
        size_t level;
        bool isLeaf;
    } visitor(*this, level, isLeaf);

    bv->accept(visitor);
}

void BVHRenderModel::addSphere(BVSphere* sphere, size_t level, bool isLeaf)
{
    double radius = sphere->getRadius();

    while (mRenderPolygons.size() <= level)
    {
        mRenderPolygons.push_back(std::vector<std::shared_ptr<SphereData>>());
    }

    std::shared_ptr<SphereData> sphereData =
            std::make_shared<SphereData>(
                mRenderModelManager,
                nullptr, sphere, level, radius, isLeaf);

    mRenderPolygons[level].push_back(sphereData);
}

BVHRenderModel::SphereData::SphereData(
        RenderModelManager* renderModelManager,
        BVSphere* bvSphere,
        int level,
        double radius,
        int resolution,
        bool isLeaf)
    : mBvSphere(bvSphere)
    , mIsLeaf(isLeaf)
    , mLevel(level)
    , mInitialized(false)
{
    mGeometricSphere = std::make_shared<GeometricSphere>(radius, resolution);

    mRenderModel = std::make_shared<PolygonRenderModel>(
                renderModelManager,
                mGeometricSphere->getPolygon());
}

BVHRenderModel::SphereData::SphereData(
        RenderModelManager* renderModelManager,
        GeometricSphere* geometricSphereTemplate,
        BVSphere* bvSphere,
        int level,
        double radius,
        bool isLeaf)
    : mBvSphere(bvSphere)
    , mIsLeaf(isLeaf)
    , mLevel(level)
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

BVHRenderModel::SphereData::~SphereData()
{

}

void BVHRenderModel::SphereData::initialize(
        Renderer* renderer,
        RenderModelManager* renderModelManager,
        GeometricSphere* geometricSphereTemplate)
{
    if (mInitialized)
        return;

    mInitialized = true;
    mGeometricSphere = std::make_shared<GeometricSphere>(*geometricSphereTemplate);
    mGeometricSphere->getPolygon()->getTransform().scale(mRadius*25);
//    setRadius(mRadius);
    mRenderModel = std::make_shared<PolygonRenderModel>(
                renderModelManager,
                mGeometricSphere->getPolygon());
    mRenderModel->addToRenderer(renderer);

    update();
}

void BVHRenderModel::SphereData::update()
{
    if (mRenderModel)
    {
        setRadius(mBvSphere->getRadius());
        setPosition(mBvSphere->getPosition());
        mRenderModel->update();
    }
}

void BVHRenderModel::SphereData::addToRenderer(Renderer* renderer)
{
    if (mRenderModel)
        mRenderModel->addToRenderer(renderer);
}

void BVHRenderModel::SphereData::removeFromRenderer(Renderer* renderer)
{
    if (mRenderModel)
        mRenderModel->removeFromRenderer(renderer);
}

void BVHRenderModel::SphereData::setVisible(bool visible, int level)
{
    if (!mRenderModel)
        return;

    mRenderModel->setVisible(isToBeSetVisible(visible, level));
}

bool BVHRenderModel::SphereData::isToBeSetVisible(bool visible, int level)
{
    if (level == -1) // Render all
    {
        return visible;
    }
    else if (level == -2) // Render leafs
    {
        return visible && mIsLeaf;
    }
    else // Render the corresponding level
    {
        return visible && level == static_cast<int>(mLevel);
    }
}

void BVHRenderModel::SphereData::setRadius(double radius)
{
    mRadius = radius;
    if (mGeometricSphere)
    {
        if (std::abs(mRadiusPrevious - mBvSphere->getRadius()) > 1e-5)
        {
            mGeometricSphere->setRadiusAndProject(radius);
            mGeometricSphere->getPolygon()->update();

            mRadiusPrevious = mBvSphere->getRadius();
        }
    }
}

void BVHRenderModel::SphereData::setPosition(Eigen::Vector position)
{
    mGeometricSphere->getPolygon()->getTransform().translation() = position;
    mGeometricSphere->getPolygon()->update();
}
