#include "SelectionRectangle.h"
#include "SelectionSceneData.h"

#include <scene/scene_graph/SceneData.h>
#include <scene/scene_graph/SceneLeafData.h>

#include <rendering/ViewFrustum.h>

#include <scene/data/GeometricData.h>

SelectionSceneData::SelectionSceneData()
{

}

SelectionSceneData::~SelectionSceneData()
{

}

std::set<SceneData*>& SelectionSceneData::getSceneData()
{
    return mSceneDatas;
}

void SelectionSceneData::clear()
{
    mSceneDatas.clear();
}

void SelectionSceneData::updateSelectionByRectangle(
        SceneLeafData* leafData,
        ViewFrustum* viewFrustum,
        SelectionRectangle& rectangle)
{
    GeometricData* gd = leafData->getGeometricDataRaw();
    for (ID i = 0; i < gd->getSize(); ++i)
    {
        Vector& v = gd->getPosition(i);
        if (rectangle.testVertex(v, viewFrustum))
        {
            mSceneDatas.insert(leafData);
        }
    }

}

void SelectionSceneData::updateSelectionByRay(
        SceneLeafData* leafData,
        ViewFrustum* viewFrustum,
        int x,
        int y)
{

}
