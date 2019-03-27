#ifndef TREE_H
#define TREE_H

// Includes
// Here are all the includes of this library to simplify
// usage of the library in general.
#include "ChildrenNode.h"
#include "factories/TreeFactory.h"
#include "factories/TreeNodeFactory.h"
#include "LeafNode.h"
#include "listeners/NodeListener.h"
#include "listeners/ChildrenNodeListener.h"
#include "listeners/LeafNodeListener.h"
#include "Node.h"
#include "NodeVisitor.h"
#include "NodeVisitorImpl.h"
#include "TraversalFilter.h"
#include "TreeTraverser.h"
#include <string>


// Forward Declarations
template <class T, class L>
class ChildrenNode;

template <class T, class L>
class TreeListener;

// This is a wrapper class for a top level children node. It catches
// all events that are send to all nodes of the subtree and them to
// every registered TreeListener.
// Call the setTree method once on the root node.
// When adding children to the root (addChild), it automatically regsiters
// them to this tree. This does not need to be done manually, therefore,
// a call of the method setTree() is only necessary once.
template <class T, class L>
class Tree : public NodeListener<T, L>,
        public ChildrenNodeListener<T, L>,
        public LeafNodeListener<T, L>
{
public:
    Tree(std::string name = "");
    ~Tree();

    // Getters
    ChildrenNode<T, L>* getRoot();
    std::string getName();

    // Setters
    void setRoot(ChildrenNode<T, L>* root);
    void setString(std::string name);

    // Listener Support
    void addTreeListener(TreeListener<T, L>* listener);
    void removeTreeListener(TreeListener<T, L>* listener);

    // Node Listener
    virtual void notifyParentChanged(Node<T, L>* source, Node<T, L>* parent);
    virtual void notifyNameChanged(Node<T, L>* source, std::string name);
    virtual void notifyTreeChanged(Node<T, L>* source, Tree<T, L>* tree);

    // ChildrenNodeListener
    virtual void notifyChildAdded(Node<T, L>* source, Node<T, L>* childNode);
    virtual void notifyChildRemoved(Node<T, L>* source, Node<T, L>* childNode);
    virtual void notifyChildrenDataChanged(Node<T, L>* source, T& data);

    // LeafNodeListener
    virtual void notifyLeafDataChanged(Node<T, L>* source, L& data);

private:
    ChildrenNode<T, L>* mRoot;

    std::vector<TreeListener<T, L>*> mListeners;

    std::string mName;


    };

#include "Tree.cpp"

#endif // TREE_H
