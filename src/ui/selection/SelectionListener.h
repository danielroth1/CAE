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

    virtual void onSceneNodeSelected(SceneData* sd) = 0;

    virtual void onSelectedSceneNodesChanged(
            const std::set<SceneData*>& sd) = 0;

    virtual void onSelectedVerticesChanged(
            const std::map<SceneLeafData*, std::vector<ID>>& sv) = 0;
};

#endif // SELECTIONLISTENER_H
