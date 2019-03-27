#ifndef LEAFNODE_CPP
#define LEAFNODE_CPP

// Includes
#include "LeafNode.h"
#include "NodeVisitor.h"
#include <data_structures/tree/listeners/LeafNodeListener.h>

#include <algorithm>

template <class T, class L>
LeafNode<T, L>::LeafNode(std::string name)
    : Node<T, L>(name)
{
}

template <class T, class L>
LeafNode<T, L>::LeafNode(ChildrenNode<T, L>* parent, std::string name)
    : Node<T, L>(parent, name)
{

}

template <class T, class L>
LeafNode<T, L>::~LeafNode()
{
}

template <class T, class L>
void LeafNode<T, L>::accept(NodeVisitor<T, L>& visitor)
{
    visitor.visit(this);
}

template<class T, class L>
void LeafNode<T, L>::registerTree(Tree<T, L>* tree)
{
    LeafNode<T, L>::setTree(tree);
}

template<class T, class L>
L& LeafNode<T, L>::getData()
{
    return mData;
}

template<class T, class L>
std::size_t LeafNode<T, L>::getNumberOfChildren() const
{
    return 0;
}

template<class T, class L>
bool LeafNode<T, L>::isLeaf() const
{
    return true;
}

template<class T, class L>
void LeafNode<T, L>::setTree(Tree<T, L>* tree)
{
    if (Node<T, L>::mTree)
        removeListener(Node<T, L>::mTree);
    if (tree)
        addListener(tree);
    Node<T, L>::setTree(tree);
}

template<class T, class L>
void LeafNode<T, L>::setData(L data)
{
    mData = data;

    for (LeafNodeListener<T, L>* l : mListeners)
        l->notifyLeafDataChanged(this, mData);
}

template<class T, class L>
void LeafNode<T, L>::addListener(LeafNodeListener<T, L>* listener)
{
    mListeners.push_back(listener);
}

template<class T, class L>
void LeafNode<T, L>::removeListener(LeafNodeListener<T, L>* listener)
{
    auto it = std::find(mListeners.begin(), mListeners.end(), listener);
    if (it != mListeners.end())
        mListeners.erase(it);
}


#endif // LEAFNODE_CPP
