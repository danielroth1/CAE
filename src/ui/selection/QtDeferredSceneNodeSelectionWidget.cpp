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
        if (getButtonSetSelection()->isChecked())
        {
            std::vector<SGNode*> nodes;
            std::vector<std::string> names;
            nodes.push_back(static_cast<SGNode*>(sd->getNode()));
            names.push_back(sd->getNode()->getName());
            selectElementsIfButtonPressed(nodes, names);
            updateListeners();
        }
    }
}

void QtDeferredSceneNodeSelectionWidget::onSelectedSceneNodesChanged(
        const std::set<std::shared_ptr<SceneData> >& sds)
{
    if (getButtonSetSelection()->isChecked())
    {
        std::vector<SGNode*> nodes;
        std::vector<std::string> names;
        for (const std::shared_ptr<SceneData>& sd : sds)
        {
            SGNode* node = static_cast<SGNode*>(sd->getNode());
            nodes.push_back(node);
            names.push_back(sd->getNode()->getName());
        }
        selectElementsIfButtonPressed(nodes, names);
        updateListeners();
    }
}

void QtDeferredSceneNodeSelectionWidget::onSelectedVerticesChanged(
        const std::map<std::shared_ptr<SceneLeafData>,
        std::vector<ID> >& /*sv*/)
{
    // Nothing to do here.
}

void QtDeferredSceneNodeSelectionWidget::notifyParentChanged(SGNode* /*source*/, SGNode* /*parent*/)
{
    // Nothing to do here.
}

void QtDeferredSceneNodeSelectionWidget::notifyNameChanged(SGNode* source, std::string name)
{
    updateName(getIndex(source), name);
}

void QtDeferredSceneNodeSelectionWidget::notifyTreeChanged(SGNode* /*source*/, SGTree* /*tree*/)
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
        updateListeners();
    }
}

void QtDeferredSceneNodeSelectionWidget::changeSelectionClickedSlot(bool /*checked*/)
{
    mSelectionControl->selectSceneNode(getSelectedElement());
}

void QtDeferredSceneNodeSelectionWidget::updateListeners()
{
    for (SGNode* node : mListenedSceneNodes)
    {
        node->removeListener(this);
    }
    mListenedSceneNodes = getSelectedElements();
    for (SGNode* node : mListenedSceneNodes)
    {
        node->addListener(this);
    }
}
