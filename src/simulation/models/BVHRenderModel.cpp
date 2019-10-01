#include "BVHRenderModel.h"
#include "BVHRenderModelSphereData.h"

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
    for (std::vector<std::shared_ptr<BVHRenderModelSphereData>>& datas : mRenderPolygons)
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
    if (!isVisible())
        return;

    // Update only the specific visible level. If all levels are visible or all
    // leafs, just try to update all. Invisible ones are ignored while iterating.
    if (mRenderedLevel >= 0)
    {
        const std::vector<std::shared_ptr<BVHRenderModelSphereData>>& levels =
                mRenderPolygons[static_cast<size_t>(mRenderedLevel)];

        for (const std::shared_ptr<BVHRenderModelSphereData>& BVHRenderModelSphereData : levels)
        {
            if (BVHRenderModelSphereData->isVisible())
            {
                BVHRenderModelSphereData->update();
            }
        }
    }
    else
    {
        for (const std::vector<std::shared_ptr<BVHRenderModelSphereData>>& levels : mRenderPolygons)
        {
            for (const std::shared_ptr<BVHRenderModelSphereData>& sphereData : levels)
            {
                if (sphereData->isVisible())
                {
                    sphereData->update();
                }
            }
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
    for (const std::vector<std::shared_ptr<BVHRenderModelSphereData>>& levels : mRenderPolygons)
    {
        for (const std::shared_ptr<BVHRenderModelSphereData>& sphereData : levels)
        {
            sphereData->addToRenderer(renderer);
        }
    }
}

void BVHRenderModel::removeFromRenderer(Renderer* renderer)
{
    for (const std::vector<std::shared_ptr<BVHRenderModelSphereData>>& levels : mRenderPolygons)
    {
        for (const std::shared_ptr<BVHRenderModelSphereData>& sphereData : levels)
        {
            sphereData->removeFromRenderer(renderer);
        }
    }
}

void BVHRenderModel::setVisible(bool visible)
{
    for (size_t i = 0; i < mRenderPolygons.size(); ++i)
    {
        const std::vector<std::shared_ptr<BVHRenderModelSphereData>>& levelData = mRenderPolygons[i];
        for (const std::shared_ptr<BVHRenderModelSphereData>& sphereData : levelData)
        {
            if (sphereData->isToBeSetVisible(visible, mRenderedLevel))
            {
                sphereData->initialize(mRenderer,
                                       mRenderModelManager,
                                       mGeometricSphereTemplate.get());
            }
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

        virtual void visit(BVAABB* aabb)
        {

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
        mRenderPolygons.push_back(
                    std::vector<std::shared_ptr<BVHRenderModelSphereData>>());
    }

    std::shared_ptr<BVHRenderModelSphereData> sphereData =
            std::make_shared<BVHRenderModelSphereData>(
                mRenderModelManager,
                nullptr, sphere, level, radius, isLeaf);

    mRenderPolygons[level].push_back(sphereData);
}
