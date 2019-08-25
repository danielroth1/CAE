#ifndef SELECTIONSCENEDATA_H
#define SELECTIONSCENEDATA_H

#include "Selection.h"

#include <set>

class SceneData;

// Manages the selection of SceneData objects
class SelectionSceneData: public Selection
{
public:
    SelectionSceneData();
    virtual ~SelectionSceneData() override;

    const std::set<std::shared_ptr<SceneData>>& getSceneData() const;

    // Updates the selection with the given sceneData.
    // Adds them to selection is shift is pressed.
    // Removes them from selection if ctrl is pressed.
    // Clears the previous selection if nothing is presed.
    void updateSelection(const std::set<std::shared_ptr<SceneData>>& sceneData);

    // Determines the selected scene data by screen rectangle. Adds them
    // to selectionOut.
    void calculateSelectionByRectangle(
            const std::shared_ptr<SceneLeafData>& leafData,
            ViewFrustum* viewFrustum,
            SelectionRectangle& rectangle,
            std::set<std::shared_ptr<SceneData>>& selectionOut) const;

    // Determines the selected scene data by ray casting. Adds them to
    // selectionOut.
    void calculateSelectionByRay(
            const std::shared_ptr<SceneLeafData>& leafData,
            ViewFrustum* viewFrustum,
            int x,
            int y,
            std::set<std::shared_ptr<SceneData>>& selectionOut) const;

    // Selection interface
public:

    virtual void clear() override;

private:
    std::set<std::shared_ptr<SceneData>> mSceneDatas;

};

#endif // SELECTIONSCENEDATA_H
