#include "GeometricDataListener.h"
#include "MeshInterpolationManager.h"
#include "MeshInterpolator.h"
#include "MeshInterpolatorFEM.h"
#include "MeshInterpolatorMeshMesh.h"
#include "AbstractPolygon.h"

#include <scene/model/MeshInterpolatorRenderModel.h>

#include <ApplicationControl.h>

MeshInterpolationManager::MeshInterpolationManager(Renderer* renderer)
    : mRenderer(renderer)
{

}

std::shared_ptr<MeshInterpolatorFEM> MeshInterpolationManager::addInterpolatorFEM(
        const std::shared_ptr<Polygon3D>& source,
        const std::shared_ptr<AbstractPolygon>& target)
{
    std::shared_ptr<MeshInterpolationData> data = getData(target);
    if (data != nullptr)
    {
        std::cout << "Cannot add interpolation for target AbstractPolygon because there"
                     " is already one.\n";
        return nullptr;
    }

    std::shared_ptr<MeshInterpolatorFEM> interpolator =
            std::make_shared<MeshInterpolatorFEM>(source, target);
    interpolator->solve();

    addInterpolator(interpolator);
    return interpolator;
}

std::shared_ptr<MeshInterpolatorMeshMesh> MeshInterpolationManager::addInterpolatorMeshMesh(
        const std::shared_ptr<AbstractPolygon>& source,
        const std::shared_ptr<AbstractPolygon>& target)
{
    std::shared_ptr<MeshInterpolationData> data = getData(target);
    if (data != nullptr)
    {
        std::cout << "Cannot add interpolation for target AbstractPolygon because there"
                     " is already one.\n";
        return nullptr;
    }

    std::shared_ptr<MeshInterpolatorMeshMesh> interpolator =
            std::make_shared<MeshInterpolatorMeshMesh>(source, target);
    interpolator->solve();

    addInterpolator(interpolator);

    return interpolator;
}

bool MeshInterpolationManager::removeInterpolator(
        const std::shared_ptr<AbstractPolygon>& poly)
{
    std::vector<std::shared_ptr<MeshInterpolationData>> datas = getDatas(poly);
    bool success = true;

    if (datas.empty())
    {
        std::cout << "No interpolation to remove.\n";
        success = false;
    }

    for (const std::shared_ptr<MeshInterpolationData>& data : datas)
    {
        auto it = std::find(mData.begin(), mData.end(), data);
        if (it == mData.end())
        {
            std::cout << "Should not be happening. Possible race condition.\n";
            success = false;
            continue;
        }
        unload(*it);
        mData.erase(it);
    }

    return success;
}

bool MeshInterpolationManager::removeInterpolatorByTarget(
        const std::shared_ptr<AbstractPolygon>& polyTarget)
{
    std::shared_ptr<MeshInterpolationData> data = getData(polyTarget);
    if (data)
    {
        auto it = std::find(mData.begin(), mData.end(), data);
        if (it == mData.end())
        {
            std::cout << "Should not be happening. Possible race condition.\n";
            return false;
        }
        unload(data);
        mData.erase(it);
        return true;
    }
    return false;
}

void MeshInterpolationManager::clearInterpolators()
{
    if (mRenderer != nullptr)
    {
        for (const std::shared_ptr<MeshInterpolationData>& data : mData)
        {
            unload(data);
        }
    }
    mData.clear();
}

void MeshInterpolationManager::setInterpolatorVisible(const std::shared_ptr<AbstractPolygon>& target,
                                                       bool visible)
{
    std::shared_ptr<MeshInterpolatorRenderModel> rm = getRenderModel(target);
    if (rm != nullptr)
        rm->setVisible(visible);
}

void MeshInterpolationManager::setInterpolatorsVisible(bool visible)
{
    for (const std::shared_ptr<MeshInterpolationData>& data : mData)
    {
        data->mRenderModel->setVisible(visible);
    }
}

bool MeshInterpolationManager::isInterpolatorsVisible() const
{
    if (mData.empty())
        return false;

    for (const std::shared_ptr<MeshInterpolationData>& data : mData)
    {
        if (!data->mRenderModel->isVisible())
            return false;
    }
    return true;
}

std::shared_ptr<MeshInterpolatorRenderModel> MeshInterpolationManager::getRenderModel(
        const std::shared_ptr<AbstractPolygon>& target)
{
    std::shared_ptr<MeshInterpolationData> data = getData(target);
    if (data != nullptr)
    {
        return data->mRenderModel;
    }
    return nullptr;
}

std::shared_ptr<MeshInterpolator> MeshInterpolationManager::getInterpolator(
        const std::shared_ptr<AbstractPolygon>& target)
{
    std::shared_ptr<MeshInterpolationData> data = getData(target);
    if (data != nullptr)
    {
        return data->mInterpolator;
    }
    return nullptr;
}

void MeshInterpolationManager::unload(
        const std::shared_ptr<MeshInterpolationData>& data)
{
    if (mRenderer)
        data->mRenderModel->removeFromRenderer(mRenderer);
    data->mInterpolator->getSource()->removeGeometricDataListener(
                data->mListener.get());
}

std::shared_ptr<MeshInterpolatorRenderModel>
MeshInterpolationManager::createRenderModel(
        const std::shared_ptr<MeshInterpolator>& interpolator)
{
    std::shared_ptr<MeshInterpolatorRenderModel> rm =
            std::make_shared<MeshInterpolatorRenderModel>(
                interpolator, false, true);

    rm->addToRenderer(mRenderer);
    rm->setAddedToRenderer(true);
    return rm;
}

std::shared_ptr<GeometricDataListener>
MeshInterpolationManager::createGeometricDataListener(
        const std::shared_ptr<MeshInterpolator>& interpolator,
        const std::shared_ptr<MeshInterpolatorRenderModel>& renderModel)
{
    class InterpolatorModelUpdater : public GeometricDataListener
    {
    public:
        InterpolatorModelUpdater(
                    const std::shared_ptr<MeshInterpolator>& _interpolator,
                    const std::shared_ptr<MeshInterpolatorRenderModel>& _renderModel)
            : interpolator(_interpolator)
            , renderModel(_renderModel)
        {

        }

        virtual void notifyGeometricDataChanged()
        {
            interpolator->update();
            renderModel->update();
        }

        std::shared_ptr<MeshInterpolator> interpolator;
        std::shared_ptr<MeshInterpolatorRenderModel> renderModel;
    };

    std::shared_ptr<GeometricDataListener> listener =
            std::make_shared<InterpolatorModelUpdater>(
                interpolator, renderModel);

    interpolator->getSource()->addGeometricDataListener(listener);

    return listener;
}

std::shared_ptr<MeshInterpolationManager::MeshInterpolationData>
MeshInterpolationManager::getData(
        const std::shared_ptr<AbstractPolygon>& target)
{
    for (const std::shared_ptr<MeshInterpolationData>& data : mData)
    {
        if (data->mInterpolator->getTarget() == target)
        {
            return data;
        }
    }
    return nullptr;
}

std::vector<std::shared_ptr<MeshInterpolationManager::MeshInterpolationData>>
MeshInterpolationManager::getDatas(const std::shared_ptr<AbstractPolygon>& polygon)
{
    std::vector<std::shared_ptr<MeshInterpolationData>> datas;
    for (const std::shared_ptr<MeshInterpolationData>& data : mData)
    {
        if (data->mInterpolator->getTarget() == polygon ||
            data->mInterpolator->getSource() == polygon)
        {
            datas.push_back(data);
        }
    }
    return datas;
}

void MeshInterpolationManager::addInterpolator(
        const std::shared_ptr<MeshInterpolator>& interpolator)
{
    std::shared_ptr<MeshInterpolationData> data =
            std::make_shared<MeshInterpolationData>();

    data->mInterpolator = interpolator;

    data->mRenderModel = createRenderModel(data->mInterpolator);
    data->mListener = createGeometricDataListener(data->mInterpolator,
                                                  data->mRenderModel);

    mData.push_back(data);
}
