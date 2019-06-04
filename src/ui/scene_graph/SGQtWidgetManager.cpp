#include "SGQtWidgetManager.h"

#include <scene/model/PolygonRenderModel.h>
#include <scene/model/RenderModel.h>
#include <scene/model/RenderModelVisitor.h>


SGQtWidgetManager::SGQtWidgetManager(QTreeWidget* treeWidget)
    : mTreeWidget(treeWidget)
{
    mVisualizeFaceNormals = false;
    mVisualizeVertexNormals = false;
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

SGNode *SGQtWidgetManager::get(QTreeWidgetItem* item)
{
    return mBidirectionalMap.get(item);
}

QTreeWidgetItem* SGQtWidgetManager::get(SGNode* node)
{
    return mBidirectionalMap.get(node);
}

void SGQtWidgetManager::addNode(SGNode* node)
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

void SGQtWidgetManager::removeNode(SGNode* node)
{
    QTreeWidgetItem* item = mBidirectionalMap.get(node);
    item->parent()->removeChild(item);
    delete item; // TOOD: is this correct?
    // remove from bidirectional map
    mBidirectionalMap.remove(node);
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
