#ifndef TREETRAVERSER_H
#define TREETRAVERSER_H

// Includes
//#include "ChildrenNode.h"
//#include "Node.h"
//#include "NodeVisitor.h"
//#include "TraversalFilter.h"
#include <stack>
#include <tuple>
#include <vector>

// Forward Declarations
template <class T, class L>
class ChildrenNode;

template <class T, class L>
class LeafNode;

template <class T, class L>
class Node;

template <class T, class L>
class NodeVisitor;

template <class T, class L>
class TraversalFilter;

enum TraverserStrategy
{
    TOP_TO_BOTTOM, BOTTOM_TO_TOP
};

// Class TreeTraverser
template <class T, class L>
class TreeTraverser : public NodeVisitor<T, L>
{
public:

    // Nested TreeIterator class
    // Not a typical iterator as it is known in the stl.
    // Saves only current state. Use next() method of TreeTraverser class to
    // iterate over tree.
    class Iterator
    {
    public:
        Iterator(Node<T, L>* currentNode, int childCount);
        ~Iterator();

        int getChildCount() const;
        Node<T, L>* getCurrentNode() const;
        bool operator==(Iterator const& it) const;

        Node<T, L> * operator->();
        Node<T, L> const * operator->() const;

        Node<T, L> & operator*();
        Node<T, L> operator*() const;

    private:
        Node<T, L>* mCurrentNode;
        int mChildCount;
    };

    TreeTraverser(Node<T, L>* root);

    TreeTraverser(Node<T, L>* root, TraversalFilter<T, L>* filter);

    ~TreeTraverser();

    void traverse(NodeVisitor<T, L>& nodeVisitor, TraverserStrategy strategy = TOP_TO_BOTTOM);

    Node<T, L>* begin();
    Node<T, L>* end();
    Node<T, L>* next();

    TraversalFilter<T, L>* getFilter() const;
    void setFilter(TraversalFilter<T, L>* filter);

    size_t getCurrentLevel() const;

private:
    // Node Visitor

    // Iteration constraint:
    // After each iteratoin it must hold:
    // The current node must be a parent of a child that has not been visited
    virtual void visit(ChildrenNode<T, L>* childrenNode);
    virtual void visit(LeafNode<T, L>* leafNode);

    void goStepDown(ChildrenNode<T, L>* childrenNode);
    void goStepUp();

    std::stack<unsigned int> mStack;

    TraversalFilter<T, L>* mFilter;

    Node<T, L>* mRoot;
    Node<T, L>* mCurrentNode;
    unsigned int mCurrentChildId;
    TraverserStrategy mCurrentStrategy;
};


#include "TreeTraverser.cpp"

#endif // TREETRAVERSER_H
