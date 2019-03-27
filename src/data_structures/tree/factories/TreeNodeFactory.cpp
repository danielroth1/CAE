#ifndef TREENODEFACTORY_CPP
#define TREENODEFACTORY_CPP

// Includes
#include "TreeNodeFactory.h"

#include "data_structures/tree/ChildrenNode.h"
#include "data_structures/tree/LeafNode.h"
#include "data_structures/tree/Node.h"

template <class T, class L>
ChildrenNode<T, L>* TreeNodeFactory<T, L>::createChildrenNode(ChildrenNode<T, L> *parent, std::string name)
{
    ChildrenNode<T, L>* node = new ChildrenNode<T, L>(parent, name);
    parent->addChild(node);
    return node;
}

template <class T, class L>
LeafNode<T, L>* TreeNodeFactory<T, L>::createLeafNode(ChildrenNode<T, L> *parent, std::string name)
{
    LeafNode<T, L>* node = new LeafNode<T, L>(parent, name);
    parent->addChild(node);
    return node;
}
#endif // TREENODEFACTORY_CPP
