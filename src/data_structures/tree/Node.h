#pragma once

#ifndef NODE_H
#define NODE_H

// Includes
#include <string>
#include <vector>
//#include "ChildrenNode.h"
//#include "NodeVisitor.h"

// Forward Declarations
template <class T, class L>
class NodeVisitor;

template <class T, class L>
class ChildrenNode;

template <class T, class L>
class NodeListener;

template <class T, class L>
class Tree;

template <class T, class L>
class Node
{
public:
    // Default constructor
    // @param name - name of the node
    // sets parent = NULL    
    Node(std::string name = "", Tree<T, L>* tree = nullptr);

    // Constructor with parent node
    Node(ChildrenNode<T, L>* parent, std::string name = "", Tree<T, L>* tree = nullptr);

    virtual ~Node();

    // Calculate the depth of this node in the tree by traversing up the tree.
    // If this node has no parent (aka its the root node) the depth is zero.
    size_t calculateDepth();

    // Visitor Pattern
    virtual void accept(NodeVisitor<T, L>& visitor) = 0;

    virtual bool isLeaf() const = 0;

    virtual std::size_t getNumberOfChildren() const = 0;

    // Accessors
    virtual Node<T, L>* searchNodeByName(std::string name);

    // Getters
    virtual ChildrenNode<T, L>* getParent();
    // Returns the name of the node
    virtual std::string getName() const;
    virtual Tree<T, L>* getTree();


    // Register the given tree to this node and the whole subgraph.
    // Each node removes itself as listener from the old tree and
    // registers itself to the given tree.
    // Use this method instead of setTree() to guarantee consistency,
    // i.e. all nodes of the same graph point to the same tree and
    // that tree listens to all events of the nodes.
    virtual void registerTree(Tree<T, L>* tree) = 0;

    // Setters
    // Sets the tree and adds it as listener to this node.
    // If this node already has a tree, it is removed as observer
    // before overriting the pointer.
    // Sets the tree for the whole sub tree as well.
    // Don't call this method ever! It is called automatically,
    // when this node is addeded as a child to one node that is
    // part of the tree.
    virtual void setTree(Tree<T, L>* tree);
    virtual void setParent(ChildrenNode<T, L>* parent);
    virtual void setName(std::string name);

    // Listener Supporta
    virtual void addListener(NodeListener<T, L>* listener);
    virtual void removeListener(NodeListener<T, L>* listener);

protected:
    ChildrenNode<T, L>* mParent;

    std::string mName;

    Tree<T, L>* mTree;

    std::vector<NodeListener<T, L>*> mListeners;
};

#include "Node.cpp"

#endif // NODE_H
