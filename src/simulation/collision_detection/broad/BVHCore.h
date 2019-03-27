#ifndef BVHCORE_H
#define BVHCORE_H

#include "BVChildrenData.h"
#include "BVLeafData.h"

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


typedef ChildrenNode<BVChildrenData*, BVLeafData*>          BVHChildrenNode;
typedef LeafNode<BVChildrenData*, BVLeafData*>              BVHLeafNode;
typedef Node<BVChildrenData*, BVLeafData*>                  BVHNode;
typedef NodeVisitor<BVChildrenData*, BVLeafData*>           BVHNodeVisitor;
typedef NodeVisitorImpl<BVChildrenData*, BVLeafData*>       BVHNodeVisitorImpl;
typedef TraversalFilter<BVChildrenData*, BVLeafData*>       BVHFilter;
//typedef Tree<BVChildrenData*, BVLeafData*>                  BVHSceneGraph;
typedef TreeTraverser<BVChildrenData*, BVLeafData*>         BVHTreeTraverser;
typedef ChildrenNodeListener<BVChildrenData*, BVLeafData*>  BVHChildrenNodeListener;
typedef LeafNodeListener<BVChildrenData*, BVLeafData*>      BVHLeafNodeListener;
typedef NodeListener<BVChildrenData*, BVLeafData*>          BVHNodeListener;
typedef TreeListener<BVChildrenData*, BVLeafData*>          BVHTreeListener;
typedef TreeFactory<BVChildrenData*, BVLeafData*>           BVHTreeFactory;
typedef TreeNodeFactory<BVChildrenData*, BVLeafData*>       BVHTreeNodeFactory;

class BVHCore
{
public:
    static BoundingVolume* getBoundingVolume(BVHNode* node);

public:
    class GetBoundingVolumeVisitor : public BVHNodeVisitor
    {
    public:

        virtual void visit(BVHChildrenNode* childrenNode) override;

        virtual void visit(BVHLeafNode* leafNode) override;

        BoundingVolume* boundingVolume;
    };

    static GetBoundingVolumeVisitor mGetBoundingVolumeVisitor;
};

#endif // BVHCORE_H
