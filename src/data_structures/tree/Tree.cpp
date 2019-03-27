#ifndef TREE_CPP
#define TREE_CPP

#include "Tree.h"

#include "ChildrenNode.h"
#include "listeners/TreeListener.h"

#include <algorithm>

template <class T, class L>
Tree<T, L>::Tree(std::string name)
    : mName(name)
{
    mRoot = nullptr;
    setRoot(new ChildrenNode<T, L>("root"));
}

template <class T, class L>
Tree<T, L>::~Tree()
{
    delete mRoot;
}

template <class T, class L>
ChildrenNode<T, L>* Tree<T, L>::getRoot()
{
    return mRoot;
}

template <class T, class L>
std::string Tree<T, L>::getName()
{
    return mName;
}

template<class T, class L>
void Tree<T, L>::setRoot(ChildrenNode<T, L>* root)
{
    if (mRoot)
    {
        mRoot->registerTree(nullptr);
        delete mRoot;
    }
    mRoot = root;
    root->registerTree(this);
}

template <class T, class L>
void Tree<T, L>::setString(std::string name)
{
    mName = name;
}

template<class T, class L>
void Tree<T, L>::addTreeListener(TreeListener<T, L>* listener)
{
    mListeners.push_back(listener);
}

template<class T, class L>
void Tree<T, L>::removeTreeListener(TreeListener<T, L>* listener)
{
    auto it = std::find(mListeners.begin(),
                        mListeners.end(),
                        listener);
    if (it != mListeners.end())
        mListeners.erase(it);
}

template<class T, class L>
void Tree<T, L>::notifyParentChanged(Node<T, L>* source, Node<T, L>* parent)
{
    for (TreeListener<T, L>* l : mListeners)
        l->notifyParentChanged(source, parent);
}

template<class T, class L>
void Tree<T, L>::notifyNameChanged(Node<T, L>* source, std::string name)
{
    for (TreeListener<T, L>* l : mListeners)
        l->notifyNameChanged(source, name);
}

template<class T, class L>
void Tree<T, L>::notifyTreeChanged(Node<T, L>* source, Tree<T, L>* tree)
{
    for (TreeListener<T, L>* l : mListeners)
        l->notifyTreeChanged(source, tree);
}

template<class T, class L>
void Tree<T, L>::notifyChildAdded(Node<T, L>* source, Node<T, L>* childNode)
{
    for (TreeListener<T, L>* l : mListeners)
        l->notifyChildAdded(source, childNode);
}

template<class T, class L>
void Tree<T, L>::notifyChildRemoved(Node<T, L>* source, Node<T, L>* childNode)
{
    for (TreeListener<T, L>* l : mListeners)
        l->notifyChildRemoved(source, childNode);
}

template<class T, class L>
void Tree<T, L>::notifyChildrenDataChanged(Node<T, L>* source, T& data)
{
    for (TreeListener<T, L>* l : mListeners)
        l->notifyChildrenDataChanged(source, data);
}

template<class T, class L>
void Tree<T, L>::notifyLeafDataChanged(Node<T, L>* source, L& data)
{
    for (TreeListener<T, L>* l : mListeners)
        l->notifyLeafDataChanged(source, data);
}

#endif // TREE_CPP
