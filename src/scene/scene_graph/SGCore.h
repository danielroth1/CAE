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


typedef ChildrenNode<SceneData*, SceneLeafData*>            SGChildrenNode;
typedef LeafNode<SceneData*, SceneLeafData*>                SGLeafNode;
typedef Node<SceneData*, SceneLeafData*>                    SGNode;
typedef NodeVisitor<SceneData*, SceneLeafData*>             SGNodeVisitor;
typedef NodeVisitorImpl<SceneData*, SceneLeafData*>         SGNodeVisitorImpl;
typedef TraversalFilter<SceneData*, SceneLeafData*>         SGFilter;
typedef Tree<SceneData*, SceneLeafData*>                    SGSceneGraph;
typedef TreeTraverser<SceneData*, SceneLeafData*>           SGTraverser;
typedef ChildrenNodeListener<SceneData*, SceneLeafData*>    SGChildrenNodeListener;
typedef LeafNodeListener<SceneData*, SceneLeafData*>        SGLeafNodeListener;
typedef NodeListener<SceneData*, SceneLeafData*>            SGNodeListener;
typedef TreeListener<SceneData*, SceneLeafData*>            SGTreeListener;
typedef TreeFactory<SceneData*, SceneLeafData*>             SGTreeFactory;
typedef TreeNodeFactory<SceneData*, SceneLeafData*>         SGTreeNodeFactory;


#endif // SGCORE_H
