#pragma once

#ifndef SGCORE_H
#define SGCORE_H

#include "SceneData.h"
#include "SceneLeafData.h"

#include <data_structures/tree/ChildrenNode.h>
#include <data_structures/tree/LeafNode.h>
#include <data_structures/tree/Node.h>
#include <data_structures/tree/NodeVisitor.h>
#include <data_structures/tree/NodeVisitorImpl.h>
#include <data_structures/tree/TraversalFilter.h>
#include <data_structures/tree/Tree.h>
#include <data_structures/tree/TreeTraverser.h>
#include <data_structures/tree/factories/TreeFactory.h>
#include <data_structures/tree/factories/TreeNodeFactory.h>


typedef ChildrenNode<std::shared_ptr<SceneData>, std::shared_ptr<SceneLeafData>>            SGChildrenNode;
typedef LeafNode<std::shared_ptr<SceneData>, std::shared_ptr<SceneLeafData>>                SGLeafNode;
typedef Node<std::shared_ptr<SceneData>, std::shared_ptr<SceneLeafData>>                    SGNode;
typedef NodeVisitor<std::shared_ptr<SceneData>, std::shared_ptr<SceneLeafData>>             SGNodeVisitor;
typedef NodeVisitorImpl<std::shared_ptr<SceneData>, std::shared_ptr<SceneLeafData>>         SGNodeVisitorImpl;
typedef TraversalFilter<std::shared_ptr<SceneData>, std::shared_ptr<SceneLeafData>>         SGFilter;
typedef Tree<std::shared_ptr<SceneData>, std::shared_ptr<SceneLeafData>>                    SGSceneGraph;
typedef TreeTraverser<std::shared_ptr<SceneData>, std::shared_ptr<SceneLeafData>>           SGTraverser;
typedef ChildrenNodeListener<std::shared_ptr<SceneData>, std::shared_ptr<SceneLeafData>>    SGChildrenNodeListener;
typedef LeafNodeListener<std::shared_ptr<SceneData>, std::shared_ptr<SceneLeafData>>        SGLeafNodeListener;
typedef NodeListener<std::shared_ptr<SceneData>, std::shared_ptr<SceneLeafData>>            SGNodeListener;
typedef Tree<std::shared_ptr<SceneData>, std::shared_ptr<SceneLeafData>>                    SGTree;
typedef TreeListener<std::shared_ptr<SceneData>, std::shared_ptr<SceneLeafData>>            SGTreeListener;
typedef TreeFactory<std::shared_ptr<SceneData>, std::shared_ptr<SceneLeafData>>             SGTreeFactory;
typedef TreeNodeFactory<std::shared_ptr<SceneData>, std::shared_ptr<SceneLeafData>>         SGTreeNodeFactory;


#endif // SGCORE_H
