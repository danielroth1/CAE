#ifndef QTDEFERREDSCENENODESELECTIONWIDGET_H
#define QTDEFERREDSCENENODESELECTIONWIDGET_H

#include "SelectionListener.h"

#include <scene/scene_graph/SGCore.h>

#include <ui/qt/QtDeferredElementSelectionWidget.h>

class SelectionControl;

// A widget that offers functionality to select scene nodes.
// See QtDeferredElementSelectionWidget for more information.
class QtDeferredSceneNodeSelectionWidget
        : public QtDeferredElementSelectionWidget<SGNode*>
        , public SelectionListener
        , public SGNodeListener
{
    Q_OBJECT

public:
    QtDeferredSceneNodeSelectionWidget(QWidget* parent = nullptr);
    virtual ~QtDeferredSceneNodeSelectionWidget();

    void init(SelectionControl* selectionControl);

    // SelectionListener interface
public:
    virtual void onSceneNodeSelected(const std::shared_ptr<SceneData>& sd) override;
    virtual void onSelectedSceneNodesChanged(const std::set<std::shared_ptr<SceneData> >& sds) override;
    virtual void onSelectedVerticesChanged(const std::map<std::shared_ptr<SceneLeafData>, std::vector<ID> >& sv) override;

    // SGNodeListener interface
public:
    virtual void notifyParentChanged(SGNode* source, SGNode* parent) override;
    virtual void notifyNameChanged(SGNode* source, std::string name) override;
    virtual void notifyTreeChanged(SGNode* source, SGTree* tree) override;

private slots:
    void setSelectionReleasedSlot(bool checked);
    void changeSelectionClickedSlot(bool checked);

private:

    // Stores the scene nodes that are currently listened to.
    std::vector<SGNode*> mListenedSceneNodes;

    // Removes all listeners from all scene nodes and adds the ones needed
    // for the selected scene nodes.
    void updateListeners();

    SelectionControl* mSelectionControl;
};

#endif // QTDEFERREDSCENENODESELECTIONWIDGET_H
