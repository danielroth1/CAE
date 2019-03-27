#include "ApplicationControl.h"
#include "RenderModelManager.h"

#include <algorithm>

#include <rendering/Renderer.h>

#include <scene/model/RenderModel.h>

#include <ui/UIControl.h>

RenderModelManager::RenderModelManager(ApplicationControl* ac)
    : mAc(ac)
{

}

void RenderModelManager::updateAllRenderModels()
{
    Renderer* renderer = mAc->getUIControl()->getRenderer();
    if (renderer)
    {
        for (std::shared_ptr<RenderModel>& renderModel : mRenderModels)
        {
            updateRenderModel(renderer, renderModel);
        }
        for (auto it : mRenderModelsMap)
        {
            updateRenderModel(renderer, it.second);
        }
    }
}

void RenderModelManager::addRenderModel(
        std::shared_ptr<RenderModel> renderModel)
{
    auto it = std::find(mRenderModels.begin(), mRenderModels.end(), renderModel);
    if (it == mRenderModels.end())
    {
        mRenderModels.push_back(renderModel);
    }
}

void RenderModelManager::removeRenderModel(
        std::shared_ptr<RenderModel> renderModel)
{
    auto it = std::find(mRenderModels.begin(), mRenderModels.end(), renderModel);
    if (it != mRenderModels.end())
    {
        mRenderModels.erase(it);
    }
}

void RenderModelManager::addRenderModelByObject(
        std::shared_ptr<void> object,
        std::shared_ptr<RenderModel> renderModel)
{
    mRenderModelsMap[object] = renderModel;
}

void RenderModelManager::removeRenderModelByObject(std::shared_ptr<void> object)
{
    auto it = mRenderModelsMap.find(object);
    if (it != mRenderModelsMap.end())
    {
        Renderer* renderer = mAc->getUIControl()->getRenderer();
        if (renderer)
        {
            it->second->removeFromRenderer(renderer);
            mRenderModelsMap.erase(it);
        }
    }
}

std::shared_ptr<RenderPolygons>
RenderModelManager::getRenderPolygons(
        std::shared_ptr<PolygonData> polygonData)
{
    return mPDataToRenderPolygonsMap[polygonData];
}

void RenderModelManager::addPolygonData(
        std::shared_ptr<PolygonData> polygonData,
        std::shared_ptr<RenderPolygons> renderPolygons)
{
    mPDataToRenderPolygonsMap[polygonData] = renderPolygons;
}

void RenderModelManager::removePolygonData(
        std::shared_ptr<PolygonData> polygon2DData)
{
    mPDataToRenderPolygonsMap.erase(polygon2DData);
}

void RenderModelManager::updateRenderModel(
        Renderer* renderer,
        const std::shared_ptr<RenderModel>& renderModel)
{
    if (!renderModel->isAddedToRenderer())
    {
        renderModel->addToRenderer(renderer);
        renderModel->setAddedToRenderer(true);
    }

    // update render models that always want to be updated
//    if (renderModel->isAlwaysUpdate())
        renderModel->update();
}
