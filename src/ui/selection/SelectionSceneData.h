#ifndef SELECTIONSCENEDATA_H
#define SELECTIONSCENEDATA_H

#include "Selection.h"

#include <set>

class SceneData;

// Manages the selection of SceneData objects
class SelectionSceneData : public Selection
{
public:
    SelectionSceneData();
    virtual ~SelectionSceneData() override;

    std::set<SceneData*>& getSceneData();

    // Selection interface
public:
    virtual void clear() override;

    virtual void updateSelectionByRectangle(
            SceneLeafData* leafData,
            ViewFrustum* viewFrustum,
            SelectionRectangle& rectangle) override;

    virtual void updateSelectionByRay(
            SceneLeafData* leafData,
            ViewFrustum* viewFrustum,
            int x,
            int y) override;

private:
    std::set<SceneData*> mSceneDatas;

};

#endif // SELECTIONSCENEDATA_H
