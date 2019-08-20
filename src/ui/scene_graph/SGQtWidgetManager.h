#ifndef SGQTWIDGETMANAGER_H
#define SGQTWIDGETMANAGER_H

#include "UISGBidirectionalMap.h"
#include "scene/scene_graph/SGCore.h"

class QTreeWidget;
class RenderModelVisitor;

// Manages the Qt component that visualizes the scene graph.
// Visibility:
// The check boxes are only updated when the visibilty is checked via the ui
// (using the slot itemChangedSlot())
class SGQtWidgetManager : public QObject, public SGTreeListener
{
    Q_OBJECT

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

public slots:
    void itemChangedSlot(QTreeWidgetItem* item, int /*count*/);

private:

    // Sets the visibility of the scene node (recursively) and updates the
    // scene graph in the UI.
    void setVisibilitySceneNode(SGNode* node, bool visible);

    // Sets the visibilty check box
    // \param item - item that contains the check box
    // \param visibility - the value the check box should be changed to
    //          VISIBLE - checked
    //          INVISIBLE - unchecked
    //          UNDEFINED - partially checked
    // \param includeSubTree - if the check boxes of all nodes of items subtree
    //          should be checked as well.
    void setVisibiltyCheckBox(
            QTreeWidgetItem* item,
            SceneData::Visibility visibility,
            bool includeSubtree);

    // Updates the visibilty check boxes of all nodes in the graph. Does so
    // according to the visibilty of the SceneData that are stored .
    // Has a complexity of O(n log n).
    // TODO: not sure if this is the way to go.
//    void updateVisibiltyCheckBoxes();

    QTreeWidget* mTreeWidget;

    SGSceneGraph* mSceneGraph;

    UISGBidirectionalMap mBidirectionalMap;

    bool mVisualizeFaceNormals;
    bool mVisualizeVertexNormals;
};

#endif // SGQTWIDGETMANAGER_H
