#ifndef SELECTIONLISTENER_H
#define SELECTIONLISTENER_H

#include <data_structures/DataStructures.h>
#include <map>
#include <set>
#include <vector>

class SceneData;
class SceneLeafData;

class SelectionListener
{
public:
    SelectionListener();
    virtual ~SelectionListener();

    // Is called whenever a scene node is selected in the scene graph. Is called
    // independently of which selection mode is active.
    virtual void onSceneNodeSelected(const std::shared_ptr<SceneData>& sd) = 0;

    // Is called whenever scene nodes are selected (with the selection box in
    // the OpenGL window).
    virtual void onSelectedSceneNodesChanged(
            const std::set<std::shared_ptr<SceneData>>& sd) = 0;

    // Is called whenever vertices are selected (with the selection box in
    // the OpenGL window).
    virtual void onSelectedVerticesChanged(
            const std::map<std::shared_ptr<SceneLeafData>, std::vector<ID>>& sv) = 0;
};

#endif // SELECTIONLISTENER_H
