#include "InterpolatorModule.h"

#include <ApplicationControl.h>

#include <ui/UIControl.h>

#include <modules/interpolator/ui/InterpolatorUIControl.h>

#include <scene/data/geometric/MeshInterpolationManager.h>
#include <scene/data/geometric/MeshInterpolatorFEM.h>
#include <scene/data/geometric/MeshInterpolatorMeshMesh.h>
#include <scene/data/geometric/Polygon.h>
#include <scene/data/geometric/Polygon3D.h>

InterpolatorModule::InterpolatorModule()
{

}

std::shared_ptr<MeshInterpolator> InterpolatorModule::addInterpolator(
        SGNode* source, SGNode* target, const MeshInterpolator::Type& type)
{
    // It is not possible to add the same node multiple times as target of
    // an interpolator, because the interpolators would conflict each other.
    for (auto it : mInterpolatorMap)
    {
        if (target == std::get<1>(it.first))
        {
            std::cout << "Target node already interpolated.\n";
            return nullptr;
        }
    }

    if (!source->isLeaf() || !target->isLeaf())
    {
        std::cout << "Interpolation targets are not leaf nodes.\n";
        return nullptr;
    }

    SGLeafNode* sourceLeaf = static_cast<SGLeafNode*>(source);
    SGLeafNode* targetLeaf = static_cast<SGLeafNode*>(target);

    std::shared_ptr<GeometricData> sourceGeometry = sourceLeaf->getData()->getGeometricData();
    std::shared_ptr<GeometricData> targetGeometry = targetLeaf->getData()->getGeometricData();

    if (sourceGeometry->getType() != GeometricData::Type::POLYGON ||
            targetGeometry->getType() != GeometricData::Type::POLYGON)
    {
        std::cout << "Interpolation targets are no polygons.\n";
        return nullptr;
    }

    std::shared_ptr<Polygon> sourcePoly = std::static_pointer_cast<Polygon>(sourceGeometry);
    std::shared_ptr<Polygon> targetPoly = std::static_pointer_cast<Polygon>(targetGeometry);

    // Create the interpolator
    std::shared_ptr<MeshInterpolator> interpolator = nullptr;

    switch (type)
    {
    case MeshInterpolator::Type::FEM:
        if (sourcePoly->getDimensionType() == Polygon::DimensionType::THREE_D)
        {
            interpolator = mAc->getMeshInterpolationManager()->addInterpolatorFEM(
                        std::static_pointer_cast<Polygon3D>(sourcePoly), targetPoly);
        }
        break;
    case MeshInterpolator::Type::MESH_MESH:
        interpolator = mAc->getMeshInterpolationManager()->addInterpolatorMeshMesh(
                    sourcePoly, targetPoly);
        break;
    }
    if (interpolator != nullptr)
    {
        mInterpolatorMap[std::make_pair(source, target)] = interpolator;
    }
    else
    {
        std::cout << "Can not add interpolator because the given geometries"
                     " are not supported by it.\n";
        return nullptr;
    }

    // Update UI
    mUIControl->addInterpolator(source, target);

    // Add listeners to react on names changes of scene nodes.
    // Does not add the same listener twice to the same node, so this is fine
    // to call for nodes that have this object already registerd as listener.
    source->addListener(this);
    target->addListener(this);

    return interpolator;
}

void InterpolatorModule::removeInterpolator(SGNode* source, SGNode* target)
{
    auto it = mInterpolatorMap.find(std::make_pair(source, target));
    if (it != mInterpolatorMap.end())
    {
        std::shared_ptr<MeshInterpolator> interpolator =
                mInterpolatorMap[std::make_pair(source, target)];

        mAc->getMeshInterpolationManager()->removeInterpolatorByTarget(
                    std::static_pointer_cast<Polygon>(
                        static_cast<SGLeafNode*>(target)->getData()->getGeometricData()));

        // Remove the listener if there is no more referenec left:
        bool sourceIsPart = false;
        bool targetIsPart = false;

        // Checks if source and target node are part of any interpolation. Can
        // only remove their listeners if there is no reference of the nodes left.
        for (auto it : mInterpolatorMap)
        {
            if (source == std::get<0>(it.first) || source == std::get<1>(it.first))
            {
                sourceIsPart = true;
            }
            if (target == std::get<0>(it.first) || target == std::get<1>(it.first))
            {
                targetIsPart = true;
            }
        }

        if (sourceIsPart)
            source->removeListener(this);

        if (targetIsPart)
            target->removeListener(this);

        // Update the UI
        mUIControl->removeInterpolator(source, target);

        mInterpolatorMap.erase(it);

        if (source->isLeaf())
        {
            std::shared_ptr<SimulationObject> so =
                    static_cast<SGLeafNode*>(source)->getData()->getSimulationObject();
            if (so)
            {
                mAc->getSimulationControl()->removeCollisionObject(so);
            }
        }
    }

}

void InterpolatorModule::removeInterpolator(SGNode* node)
{
    std::vector<std::pair<SGNode*, SGNode*>> toRemoveKeys;
    for (auto it : mInterpolatorMap)
    {
        SGNode* source = std::get<0>(it.first);
        SGNode* target = std::get<1>(it.first);
        if (node == source || node == target)
        {
            toRemoveKeys.push_back(std::make_pair(source, target));
        }
    }
    for (const auto& it : toRemoveKeys)
    {
        removeInterpolator(std::get<0>(it), std::get<1>(it));
    }
}

void InterpolatorModule::clearInterpolators()
{
    auto interpolatorMapCopy = mInterpolatorMap;
    for (auto it : interpolatorMapCopy)
    {
        const std::pair<SGNode*, SGNode*>& p = it.first;
        removeInterpolator(p.first, p.second);
    }
}

void InterpolatorModule::setInterpolatorVisible(const std::shared_ptr<Polygon>& poly, bool visible)
{
    mAc->getMeshInterpolationManager()->setInterpolatorVisible(poly, visible);
}

void InterpolatorModule::init(ApplicationControl* ac)
{
    mAc = ac;
}

void InterpolatorModule::initUI(QWidget* parent)
{
    mUIControl = std::make_shared<InterpolatorUIControl>(this);

    mUIControl->init(parent, mAc->getUIControl()->getSelectionControl());
}

void InterpolatorModule::finalize()
{
    mUIControl->finalize();
}

std::string InterpolatorModule::getName()
{
    return "Interpolator";
}

QWidget* InterpolatorModule::getWidget()
{
    return mUIControl->getWidget();
}

void InterpolatorModule::notifyParentChanged(SGNode* /*source*/, SGNode* /*parent*/)
{

}

void InterpolatorModule::notifyNameChanged(SGNode* source, std::string name)
{
    mUIControl->updateNodeName(source, name);
}

void InterpolatorModule::notifyTreeChanged(SGNode* /*source*/, SGTree* /*tree*/)
{

}
