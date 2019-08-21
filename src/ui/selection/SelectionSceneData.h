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

    const std::set<std::shared_ptr<SceneData>>& getSceneData() const;

    void insert(const std::shared_ptr<SceneData>& sceneData);

    // Selection interface
public:

    virtual void clear() override;

    virtual void updateSelectionByRectangle(
            const std::shared_ptr<SceneLeafData>& leafData,
            ViewFrustum* viewFrustum,
            SelectionRectangle& rectangle) override;

    virtual void updateSelectionByRay(
            const std::shared_ptr<SceneLeafData>& leafData,
            ViewFrustum* viewFrustum,
            int x,
            int y) override;

private:
    std::set<std::shared_ptr<SceneData>> mSceneDatas;

};

#endif // SELECTIONSCENEDATA_H
