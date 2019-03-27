#include "UISGControl.h"


UISGControl::UISGControl(QTreeWidget* treeWidget)
    : mTreeWidget(treeWidget)
{
}

void UISGControl::registerNewSceneGraph(SGSceneGraph* sceneGraph)
{
    clear();
    mSceneGraph = sceneGraph;
    revalidate();
    sceneGraph->addTreeListener(this);
}

void UISGControl::clear()
{
    mTreeWidget->clear();
    mBidirectionalMap.clear();
}

void UISGControl::revalidate()
{
    clear();
    SGTraverser tt(mSceneGraph->getRoot());
    class RevalidateVisitor : public SGNodeVisitor
    {
    public:
        RevalidateVisitor(UISGControl& u)
            : ui(u)
        {
        }
        void visitImpl(SGNode* node)
        {
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

        UISGControl& ui;
    } visitor(*this);

    tt.traverse(visitor);
}

SGNode *UISGControl::get(QTreeWidgetItem* item)
{
    return mBidirectionalMap.get(item);
}

QTreeWidgetItem* UISGControl::get(SGNode* node)
{
    return mBidirectionalMap.get(node);
}

void UISGControl::addNode(SGNode* node)
{
    if (node->getParent() == nullptr)
    {
        // root node
        QTreeWidgetItem* item = new QTreeWidgetItem(mTreeWidget);
        item->setText(0, QString::fromStdString(node->getName()));
        item->flags().setFlag(Qt::ItemFlag::ItemIsEditable, true);
        // add to bidirectional map
        mBidirectionalMap.add(item, node);
    }
    else
    {
        // update ui
        QTreeWidgetItem* parentItem =
                mBidirectionalMap.get(node->getParent());
        QTreeWidgetItem* childItem = new QTreeWidgetItem();
        childItem->setText(0, QString::fromStdString(node->getName()));
        childItem->flags().setFlag(Qt::ItemFlag::ItemIsEditable, true);
        parentItem->addChild(childItem);
        // add to bidirectional map
        mBidirectionalMap.add(childItem, node);
    }
}

void UISGControl::removeNode(SGNode* node)
{
    QTreeWidgetItem* item = mBidirectionalMap.get(node);
    item->parent()->removeChild(item);
    delete item; // TOOD: is this correct?
    // remove from bidirectional map
    mBidirectionalMap.remove(node);
}

void UISGControl::notifyLeafDataChanged(SGNode* /*source*/, SceneLeafData*& /*data*/)
{

}

void UISGControl::notifyChildAdded(SGNode* /*source*/, SGNode* childNode)
{
    addNode(childNode);
}

void UISGControl::notifyChildRemoved(SGNode* /*source*/, SGNode* childNode)
{
    removeNode(childNode);
}

void UISGControl::notifyChildrenDataChanged(SGNode* /*source*/, SceneData*& /*data*/)
{

}

void UISGControl::notifyParentChanged(SGNode* /*source*/, SGNode* /*parent*/)
{

}

void UISGControl::notifyNameChanged(SGNode* /*source*/, std::string /*name*/)
{

}

void UISGControl::notifyTreeChanged(SGNode* /*source*/, SGSceneGraph* /*tree*/)
{
    // whenever a node refers to the scene graph as tree, it can be
    // assumed that that node is also a node of the scene graph

}
