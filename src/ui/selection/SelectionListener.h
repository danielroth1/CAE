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

    virtual void onSceneNodeSelected(const std::shared_ptr<SceneData>& sd) = 0;

    virtual void onSelectedSceneNodesChanged(
            const std::set<std::shared_ptr<SceneData>>& sd) = 0;

    virtual void onSelectedVerticesChanged(
            const std::map<std::shared_ptr<SceneLeafData>, std::vector<ID>>& sv) = 0;
};

#endif // SELECTIONLISTENER_H
