#pragma once

#ifndef CHILDRENNODE_CPP
#define CHILDRENNODE_CPP

// Includes
#include "ChildrenNode.h"
#include "NodeVisitor.h"
#include "Tree.h"
#include <algorithm>
#include <data_structures/tree/listeners/ChildrenNodeListener.h>

template <class T, class L>
ChildrenNode<T, L>::ChildrenNode(std::string name)
    : Node<T, L>(false, name)
{
}

template <class T, class L>
ChildrenNode<T, L>::ChildrenNode(ChildrenNode *parent, std::string name)
    : Node<T, L>(false, parent, name)
{

}

template <class T, class L>
ChildrenNode<T, L>::~ChildrenNode()
{
    for (Node<T, L>* n : mChildren)
        delete n;
}

template <class T, class L>
Node<T, L>* ChildrenNode<T, L>::getChild(unsigned int i) const
{
    return mChildren[i];
}

template<class T, class L>
const std::vector<Node<T, L>*>& ChildrenNode<T, L>::getChildren()
{
    return mChildren;
}

template <class T, class L>
void ChildrenNode<T, L>::addChild(Node<T, L>* node)
{
    mChildren.push_back(node);
    node->setParent(this);

    // add tree for the whole subtree
    node->registerTree(Node<T, L>::mTree);

    for (ChildrenNodeListener<T, L>* l : mListeners)
        l->notifyChildAdded(this, node);
}

template<class T, class L>
void ChildrenNode<T, L>::removeChild(Node<T, L>* node)
{
    auto it = std::find(mChildren.begin(), mChildren.end(), node);
    if (it != mChildren.end())
    {
        mChildren.erase(it);

        // remove tree for the whole subtree
        node->registerTree(nullptr);

        for (ChildrenNodeListener<T, L>* l : mListeners)
            l->notifyChildRemoved(this, node);

        delete node;
    }
}

template<class T, class L>
void ChildrenNode<T, L>::clear()
{
    for (size_t i = 0; i < mChildren.size(); ++i)
    {
        Node<T, L>* node = mChildren[i];
        mChildren.erase(mChildren.begin() + i);

        // remove tree for the whole subtree
        node->registerTree(nullptr);

        for (ChildrenNodeListener<T, L>* l : mListeners)
            l->notifyChildRemoved(this, node);

        --i;
        delete node;
    }
}

template <class T, class L>
std::size_t ChildrenNode<T, L>::getNumberOfChildren() const
{
    return mChildren.size();
}

template <class T, class L>
typename std::vector<Node<T, L>*>::iterator ChildrenNode<T, L>::begin()
{
    return mChildren.begin();
}

template <class T, class L>
typename std::vector<Node<T, L>*>::iterator ChildrenNode<T, L>::end()
{
    return mChildren.end();
}

template<class T, class L>
void ChildrenNode<T, L>::registerTree(Tree<T, L>* tree)
{
    TreeTraverser<T, L> tt(this);

    class SetTreeNodeVisitor : public NodeVisitor<T, L>
    {
    public:
        SetTreeNodeVisitor(Tree<T, L>* t)
            : tree(t)
        {
        }
        void visit(ChildrenNode<T, L>* childrenNode)
        {
            childrenNode->setTree(tree);
        }
        void visit(LeafNode<T, L>* leafNode)
        {
            leafNode->setTree(tree);
        }
        Tree<T, L>* tree;
    } setTreeNodeVisitor(tree);

    tt.traverse(setTreeNodeVisitor);
}

template <class T, class L>
void ChildrenNode<T, L>::accept(NodeVisitor<T, L>& visitor)
{
    visitor.visit(this);
}

template<class T, class L>
void ChildrenNode<T, L>::setData(T data)
{
    mData = data;

    for (ChildrenNodeListener<T, L>* l : mListeners)
        l->notifyChildrenDataChanged(this, data);
}

template<class T, class L>
void ChildrenNode<T, L>::setTree(Tree<T, L>* tree)
{
    if (Node<T, L>::mTree)
        removeListener(Node<T, L>::mTree);
    if (tree)
        addListener(tree);
    Node<T, L>::setTree(tree);
}

template<class T, class L>
T ChildrenNode<T, L>::getData() const
{
    return mData;
}

template<class T, class L>
void ChildrenNode<T, L>::addListener(ChildrenNodeListener<T, L>* listener)
{
    mListeners.push_back(listener);
}

template<class T, class L>
void ChildrenNode<T, L>::removeListener(ChildrenNodeListener<T, L>* listener)
{
    auto it = std::find(mListeners.begin(), mListeners.end(), listener);
    if (it != mListeners.end())
    {
        mListeners.erase(it);
    }
}

#endif // CHILDRENNODE_CPP
