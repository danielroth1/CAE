#include "SGQtWidgetManager.h"

#include <scene/model/PolygonRenderModel.h>
#include <scene/model/RenderModel.h>
#include <scene/model/RenderModelVisitor.h>


SGQtWidgetManager::SGQtWidgetManager(QTreeWidget* treeWidget)
    : mTreeWidget(treeWidget)
{
    mVisualizeFaceNormals = false;
    mVisualizeVertexNormals = false;

    QWidget::connect(mTreeWidget, SIGNAL(itemChanged(QTreeWidgetItem*, int)),
                     this, SLOT(itemChangedSlot(QTreeWidgetItem*, int)));
}

void SGQtWidgetManager::registerNewSceneGraph(SGSceneGraph* sceneGraph)
{
    clear();
    mSceneGraph = sceneGraph;
    revalidate();
    sceneGraph->addTreeListener(this);
}

void SGQtWidgetManager::clear()
{
    mTreeWidget->clear();
    mBidirectionalMap.clear();
}

void SGQtWidgetManager::revalidate()
{
    clear();
    SGTraverser tt(mSceneGraph->getRoot());
    class RevalidateVisitor : public SGNodeVisitor
    {
    public:
        RevalidateVisitor(SGQtWidgetManager& u)
            : ui(u)
        {
        }
        void visitImpl(SGNode* node)
        {
            // We don't want to change names of nodes that are added from a
            // validation.
            ui.addNode(node);
        }
        virtual void visit(SGChildrenNode* childrenNode)
        {
            visitImpl(childrenNode);
        }
        virtual void visit(SGLeafNode* leafNode)
        {
            visitImpl(leafNode);
        }

        SGQtWidgetManager& ui;
    } visitor(*this);

    tt.traverse(visitor);
}

SGNode* SGQtWidgetManager::get(QTreeWidgetItem* item)
{
    return mBidirectionalMap.get(item);
}

QTreeWidgetItem* SGQtWidgetManager::get(SGNode* node)
{
    return mBidirectionalMap.get(node);
}

void SGQtWidgetManager::addNode(SGNode* node, bool setEditing)
{
    QTreeWidgetItem* item;
    if (node->getParent() != nullptr)
        item = new QTreeWidgetItem();
    else
        item = new QTreeWidgetItem(mTreeWidget); // root node

    item->setText(0, QString::fromStdString(node->getName()));
    item->setFlags(Qt::ItemIsEditable |
                   Qt::ItemIsEnabled |
                   Qt::ItemIsSelectable |
                   Qt::ItemIsUserCheckable |
                   Qt::ItemIsDragEnabled |
                   Qt::ItemIsDropEnabled);
    item->setCheckState(0, Qt::Checked); // enable check box

    // add to parent
    if (node->getParent() != nullptr)
    {
        QTreeWidgetItem* parentItem =
                mBidirectionalMap.get(node->getParent());
        parentItem->addChild(item);
        parentItem->setExpanded(true);
    }

    if (setEditing)
        mTreeWidget->editItem(item);

    mBidirectionalMap.add(item, node);
}

void SGQtWidgetManager::removeNode(SGNode* node)
{
    QTreeWidgetItem* item = mBidirectionalMap.get(node);
    item->parent()->removeChild(item);
    delete item; // TOOD: is this correct?
    // remove from bidirectional map
    mBidirectionalMap.remove(node);
}

void SGQtWidgetManager::enableEditing(SGNode* node)
{
    QTreeWidgetItem* item = mBidirectionalMap.get(node);
    mTreeWidget->editItem(item);
}

void SGQtWidgetManager::notifyLeafDataChanged(SGNode* /*source*/, std::shared_ptr<SceneLeafData>& /*data*/)
{

}

void SGQtWidgetManager::notifyChildAdded(SGNode* /*source*/, SGNode* childNode)
{
    addNode(childNode);
}

void SGQtWidgetManager::notifyChildRemoved(SGNode* /*source*/, SGNode* childNode)
{
    removeNode(childNode);
}

void SGQtWidgetManager::notifyChildrenDataChanged(SGNode* /*source*/, std::shared_ptr<SceneData>& /*data*/)
{

}

void SGQtWidgetManager::notifyParentChanged(SGNode* /*source*/, SGNode* /*parent*/)
{

}

void SGQtWidgetManager::notifyNameChanged(SGNode* /*source*/, std::string /*name*/)
{

}

void SGQtWidgetManager::notifyTreeChanged(SGNode* /*source*/, SGSceneGraph* /*tree*/)
{
    // whenever a node refers to the scene graph as tree, it can be
    // assumed that that node is also a node of the scene graph

}

void SGQtWidgetManager::itemChangedSlot(QTreeWidgetItem* item, int /*count*/)
{
    SGNode* node = mBidirectionalMap.get(item);
    bool visible = item->checkState(0) == Qt::CheckState::Checked;
    setVisibilitySceneNode(node, visible);
}

void SGQtWidgetManager::setVisibilitySceneNode(SGNode* node, bool visible)
{
    if (node)
    {
        QTreeWidgetItem* item = mBidirectionalMap.get(node);
        if (node->isLeaf())
        {
            SGLeafNode* leafNode = static_cast<SGLeafNode*>(node);
            leafNode->getData()->setVisible(visible);
        }
        else
        {
            SGChildrenNode* childrenNode = static_cast<SGChildrenNode*>(node);
            childrenNode->getData()->setVisibleRecoursive(visible);

        }

        // Update check boxes.
        SceneData::Visibility visibility = visible ? SceneData::Visibility::VISIBLE :
                                                     SceneData::Visibility::INVISIBLE;
        setVisibiltyCheckBox(item, visibility, true);
    }
}

void SGQtWidgetManager::setVisibiltyCheckBox(
        QTreeWidgetItem* item,
        SceneData::Visibility visibility,
        bool includeSubtree)
{
    Qt::CheckState checkState;
    if (visibility == SceneData::Visibility::VISIBLE)
        checkState = Qt::Checked;
    else if (visibility == SceneData::Visibility::INVISIBLE)
        checkState = Qt::Unchecked;
    else if (visibility == SceneData::Visibility::UNDEFINED)
        checkState = Qt::PartiallyChecked;

    item->setCheckState(0, checkState);

    if (includeSubtree)
    {
        for (int i = 0; i < item->childCount(); ++i)
        {
            setVisibiltyCheckBox(item->child(i), visibility, true);
        }
    }
}
