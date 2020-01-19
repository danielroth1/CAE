#include "QtDeferredSceneNodeSelectionWidget.h"
#include "SelectionControl.h"

QtDeferredSceneNodeSelectionWidget::QtDeferredSceneNodeSelectionWidget(
        QWidget* parent)
    : QtDeferredElementSelectionWidget<SGNode *>(parent, true)
{

    QWidget::connect(getButtonSetSelection(), SIGNAL(toggled(bool)),
                     this, SLOT(setSelectionReleasedSlot(bool)));

    QWidget::connect(getButtonChangeSelection(), SIGNAL(clicked(bool)),
                     this, SLOT(changeSelectionClickedSlot(bool)));
}

QtDeferredSceneNodeSelectionWidget::~QtDeferredSceneNodeSelectionWidget()
{

}

void QtDeferredSceneNodeSelectionWidget::init(SelectionControl* selectionControl)
{
    mSelectionControl = selectionControl;
}

void QtDeferredSceneNodeSelectionWidget::onSceneNodeSelected(
        const std::shared_ptr<SceneData>& sd)
{
    // "Select vertices" only uses this listener call.
    if (mSelectionControl->getSelectionType() == SelectionControl::SelectionType::SELECT_VERTICES)
    {
        std::vector<SGNode*> nodes;
        std::vector<std::string> names;
        nodes.push_back(static_cast<SGNode*>(sd->getNode()));
        names.push_back(sd->getNode()->getName());
        selectElementsIfButtonPressed(nodes, names);
    }
}

void QtDeferredSceneNodeSelectionWidget::onSelectedSceneNodesChanged(
        const std::set<std::shared_ptr<SceneData> >& sds)
{
    std::vector<SGNode*> nodes;
    std::vector<std::string> names;
    for (const std::shared_ptr<SceneData>& sd : sds)
    {
        nodes.push_back(static_cast<SGNode*>(sd->getNode()));
        names.push_back(sd->getNode()->getName());
    }
    selectElementsIfButtonPressed(nodes, names);
}

void QtDeferredSceneNodeSelectionWidget::onSelectedVerticesChanged(
        const std::map<std::shared_ptr<SceneLeafData>,
        std::vector<ID> >& /*sv*/)
{
    // Nothing to do here.
}

void QtDeferredSceneNodeSelectionWidget::setSelectionReleasedSlot(bool checked)
{
    if (!checked)
    {
        std::vector<SGNode*> nodes;
        std::vector<std::string> names;
        for (const std::shared_ptr<SceneData>& sd : mSelectionControl->getSelectedSceneData())
        {
            nodes.push_back(sd->getNode());
            names.push_back(nodes.back()->getName());
        }
        selectElements(nodes, names);
    }
}

void QtDeferredSceneNodeSelectionWidget::changeSelectionClickedSlot(bool /*checked*/)
{
    mSelectionControl->selectSceneNode(getSelectedElement());
}
