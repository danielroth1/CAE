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
{
    Q_OBJECT

public:
    QtDeferredSceneNodeSelectionWidget(QWidget* parent = nullptr);
    virtual ~QtDeferredSceneNodeSelectionWidget();

    void init(SelectionControl* selectionControl);

    // SelectionListener interface
public:
    void onSceneNodeSelected(const std::shared_ptr<SceneData>& sd);
    void onSelectedSceneNodesChanged(const std::set<std::shared_ptr<SceneData> >& sds);
    void onSelectedVerticesChanged(const std::map<std::shared_ptr<SceneLeafData>, std::vector<ID> >& sv);

private slots:
    void setSelectionReleasedSlot(bool checked);
    void changeSelectionClickedSlot(bool checked);

private:
    SelectionControl* mSelectionControl;
};

#endif // QTDEFERREDSCENENODESELECTIONWIDGET_H
