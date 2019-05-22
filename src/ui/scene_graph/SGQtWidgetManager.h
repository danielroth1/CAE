#ifndef SGQTWIDGETMANAGER_H
#define SGQTWIDGETMANAGER_H

#include "UISGBidirectionalMap.h"
#include "scene/scene_graph/SGCore.h"

class QTreeWidget;
class RenderModelVisitor;

// Manages the Qt component that visualizes the scene graph.
class SGQtWidgetManager : public SGTreeListener
{
public:
    SGQtWidgetManager(QTreeWidget* treeWidget);

    // Delegated methods from UISGBidirectionalMap
public:
    void registerNewSceneGraph(SGSceneGraph* sceneGraph);
    void clear();
    // Completely removes and reinitiates the tree widget.
    void revalidate();

    SGNode* get(QTreeWidgetItem* item);
    QTreeWidgetItem* get(SGNode* node);

    void addNode(SGNode* child);
    void removeNode(SGNode* node);

    // LeafNodeListener interface
public:
    virtual void notifyLeafDataChanged(SGNode* source, std::shared_ptr<SceneLeafData>& data);

    // ChildrenNodeListener interface
public:
    virtual void notifyChildAdded(SGNode* source, SGNode* childNode);
    virtual void notifyChildRemoved(SGNode* source, SGNode* childNode);
    virtual void notifyChildrenDataChanged(SGNode* source, std::shared_ptr<SceneData>& data);

    // NodeListener interface
public:
    virtual void notifyParentChanged(SGNode* source, SGNode* parent);
    virtual void notifyNameChanged(SGNode* source, std::string name);
    virtual void notifyTreeChanged(SGNode* source, SGSceneGraph* tree);

private:

    QTreeWidget* mTreeWidget;

    SGSceneGraph* mSceneGraph;

    UISGBidirectionalMap mBidirectionalMap;

    bool mVisualizeFaceNormals;
    bool mVisualizeVertexNormals;
};

#endif // SGQTWIDGETMANAGER_H
