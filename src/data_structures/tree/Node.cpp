#ifndef NODE_CPP
#define NODE_CPP


#include "Node.h"
#include "ChildrenNode.h"
#include "NodeVisitor.h"
#include "Tree.h"
#include "TreeTraverser.h"

template <class T, class L>
Node<T, L>::Node(std::string name, Tree<T, L>* tree)
    : mName(name)
    , mTree(tree)
{
    mParent = nullptr;
    if (tree)
        addListener(tree);
}

template <class T, class L>
Node<T, L>::Node(ChildrenNode<T, L> *parent, std::string name, Tree<T, L>* tree)
    : mParent(parent)
    , mName(name)
    , mTree(tree)
{
    if (tree)
        addListener(tree);
}

template<class T, class L>
size_t Node<T, L>::calculateDepth()
{
    size_t depth = 0;
    Node<T, L>* current = this;
    while (current->getParent() != nullptr)
    {
        current = current->getParent();
        ++depth;
    }
    return depth;
}

template <class T, class L>
Node<T, L>::~Node<T, L>()
{
//    if (mData)
//        delete mData;
}

template <class T, class L>
Node<T, L>* Node<T, L>::searchNodeByName(std::string name)
{
//    class NameVisitor : public NodeVisitor<T, L>
//    {
//    public:
//        NameVisitor(std::string nameIn)
//        : mName(nameIn){
//        }
//        void visit(ChildrenNode<T, L>* childrenNode)
//        {
//            if (childrenNode->getName() == mName)
//                mFoundNode = childrenNode;
//        }
//        void visit(LeafNode<T, L>* leafNode)
//        {
//            if (leafNode->getName() == mName)
//                mFoundNode = leafNode;
//        }
//        std::string mName;
//        Node<T, L>* mFoundNode;
//    };
//    TreeTraverser<T, L> traverser(this);
//    for (Node<T, L>* n = traverser.begin(); n != traverser.end(); n = traverser.next())
//    {
//        NameVisitor visitor(name);
//        n->accept(visitor);
//        if (visitor.mFoundNode != NULL)
//            return visitor.mFoundNode;
//    }
    TreeTraverser<T, L> traverser(this);
    for (Node<T, L>* n = traverser.begin(); n != traverser.end(); n = traverser.next())
    {
        if (n->getName() == name)
            return n;
    }
    return NULL;
}

template <class T, class L>
ChildrenNode<T, L>* Node<T, L>::getParent()
{
    return mParent;
}

template <class T, class L>
std::string Node<T, L>::getName() const
{
    return mName;
}

template<class T, class L>
Tree<T, L>* Node<T, L>::getTree()
{
    return mTree;
}

template<class T, class L>
void Node<T, L>::setTree(Tree<T, L>* tree)
{
    // remove other observer tree if there was one
    if (mTree)
        removeListener(mTree);
    mTree = tree;

    if (mTree)
        addListener(mTree);

    for (NodeListener<T, L>* l : mListeners)
        l->notifyTreeChanged(this, mTree);
}

template <class T, class L>
void Node<T, L>::setParent(ChildrenNode<T, L>* parent)
{
    mParent = parent;

    for (NodeListener<T, L>* l : mListeners)
        l->notifyParentChanged(this, mParent);
}

template <class T, class L>
void Node<T, L>::setName(std::string name)
{
    mName = name;

    for (NodeListener<T, L>* l : mListeners)
        l->notifyNameChanged(this, name);
}

template<class T, class L>
void Node<T, L>::addListener(NodeListener<T, L>* listener)
{
    mListeners.push_back(listener);
}

template<class T, class L>
void Node<T, L>::removeListener(NodeListener<T, L>* listener)
{
    auto it = std::find(mListeners.begin(), mListeners.end(), listener);
    if (it != mListeners.end())
        mListeners.erase(it);
}

#endif // NODE_CPP
