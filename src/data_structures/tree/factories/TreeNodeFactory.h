#ifndef TREENODEFACTORY_H
#define TREENODEFACTORY_H

// Includes
//#include "data_structures/tree/ChildrenNode.h"
//#include "data_structures/tree/LeafNode.h"
//#include "data_structures/tree/Tree.h"
#include <string>

// Forward Declarations
template <class T, class L>
class ChildrenNode;

template <class T, class L>
class LeafNode;

template <class T, class L>
class Node;

template <class T, class L>
class TreeNodeFactory
{
public:
    static ChildrenNode<T, L>* createChildrenNode(ChildrenNode<T, L>* parent, std::string name = "");

    static LeafNode<T, L>* createLeafNode(ChildrenNode<T, L>* parent, std::string name = "");

private:
    TreeNodeFactory();
};

#include "TreeNodeFactory.cpp"

#endif // TREENODEFACTORY_H
