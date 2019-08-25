#include "SelectionRectangle.h"
#include "SelectionSceneData.h"

#include <scene/scene_graph/SceneData.h>
#include <scene/scene_graph/SceneLeafData.h>

#include <rendering/ViewFrustum.h>

#include <scene/data/GeometricData.h>

#include <ui/KeyManager.h>

SelectionSceneData::SelectionSceneData()
{

}

SelectionSceneData::~SelectionSceneData()
{

}

const std::set<std::shared_ptr<SceneData> >&
SelectionSceneData::getSceneData() const
{
    return mSceneDatas;
}

void SelectionSceneData::updateSelection(
        const std::set<std::shared_ptr<SceneData>>& sceneDatas)
{
    if (!KeyManager::instance()->isShiftDown() &&
        !KeyManager::instance()->isCtrlDown())
        clear();

    for (const std::shared_ptr<SceneData>& sd : sceneDatas)
    {
        if (KeyManager::instance()->isShiftDown())
        {
            // add element
            mSceneDatas.insert(sd);
        }
        else if (KeyManager::instance()->isCtrlDown())
        {
            // remove element
            mSceneDatas.erase(sd);
        }
        else
        {
            // clear elements, then add
            mSceneDatas.insert(sd);
        }
    }
}

void SelectionSceneData::calculateSelectionByRectangle(
        const std::shared_ptr<SceneLeafData>& leafData,
        ViewFrustum* viewFrustum,
        SelectionRectangle& rectangle,
        std::set<std::shared_ptr<SceneData>>& selectionOut) const
{
    GeometricData* gd = leafData->getGeometricDataRaw();
    for (ID i = 0; i < gd->getSize(); ++i)
    {
        Vector& v = gd->getPosition(i);
        if (rectangle.testVertex(v, viewFrustum))
        {
            selectionOut.insert(leafData);
        }
    }

}

void SelectionSceneData::calculateSelectionByRay(
        const std::shared_ptr<SceneLeafData>& /*leafData*/,
        ViewFrustum* /*viewFrustum*/,
        int /*x*/,
        int /*y*/,
        std::set<std::shared_ptr<SceneData>>& /*selectionOut*/) const
{

}

void SelectionSceneData::clear()
{
    mSceneDatas.clear();
}
