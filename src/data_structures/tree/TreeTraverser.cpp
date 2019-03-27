#ifndef TREETRAVERSER_CPP
#define TREETRAVERSER_CPP

#include "TreeTraverser.h"

#include "ChildrenNode.h"
#include "LeafNode.h"
#include "Node.h"
#include "NodeVisitor.h"
#include "TraversalFilter.h"

#include <iostream>

template <class T, class L>
TreeTraverser<T, L>::TreeTraverser(Node<T, L>* root)
    : mRoot(root)
    , mCurrentNode(root)
{
    mFilter = nullptr;
    mCurrentChildId = 0;
//    mStack.push(0);
}

template <class T, class L>
TreeTraverser<T, L>::TreeTraverser(Node<T, L>* root, TraversalFilter<T, L>* filter)
    : TreeTraverser(root)
{
    mFilter = filter;
    mCurrentChildId = 0;
}

template <class T, class L>
TreeTraverser<T, L>::~TreeTraverser()
{
    if (mFilter)
        delete mFilter;
}

template <class T, class L>
void TreeTraverser<T, L>::traverse(NodeVisitor<T, L>& nodeVisitor, TraverserStrategy strategy)
{
    mCurrentStrategy = strategy;
    mCurrentChildId = 0;
    mCurrentNode = mRoot;
    for (Node<T, L>* n = begin(); n != end(); n = next())
    {
        n->accept(nodeVisitor);
    }
}

template <class T, class L>
Node<T, L>* TreeTraverser<T, L>::begin()
{
    if (mCurrentStrategy == TraverserStrategy::TOP_TO_BOTTOM || mRoot->getNumberOfChildren() == 0)
        return mRoot;
    else
        return next();
}

template <class T, class L>
Node<T, L>* TreeTraverser<T, L>::end()
{
    return NULL;
}

template <class T, class L>
Node<T, L>* TreeTraverser<T, L>::next()
{
    class TreeVisitor : public NodeVisitor<T, L>
    {
    public:
        TreeVisitor(TreeTraverser<T, L>& ttIn)
            : tt(ttIn)
        {
            continueSearch = false;
            initialStep = true;
        }

        void visit(ChildrenNode<T, L>* childrenNode)
        {
            if (tt.mCurrentChildId >= childrenNode->getNumberOfChildren())
            {
                if (!initialStep && tt.mCurrentStrategy == TraverserStrategy::BOTTOM_TO_TOP)
                    continueSearch = false;
                else
                    goStepUp(childrenNode);
            }
            else if (tt.mCurrentNode != NULL)
            {
                goStepDown(childrenNode);
            }
            initialStep = false;
        }

        void visit(LeafNode<T, L> *leafNode)
        {
            if (!initialStep && tt.mCurrentStrategy == TraverserStrategy::BOTTOM_TO_TOP)
                continueSearch = false;
            else
                goStepUp(leafNode);

            initialStep = false;
        }

        void goStepUp(Node<T, L>* childrenNode)
        {
            if (childrenNode->getParent() == NULL || tt.mStack.empty())
            {
                tt.mCurrentNode = NULL;
//                if (tt.mCurrentStrategy == TraverserStrategy::TOP_TO_BOTTOM)
                    continueSearch = false;
//                else if (tt.mCurrentStrategy == TraverserStrategy::BOTTOM_TO_TOP)
//                    continueSearch = true;
            }
            else
            {
                //std::cout << "up.\n";
                tt.goStepUp();
//                if (tt.mCurrentStrategy == TraverserStrategy::TOP_TO_BOTTOM)
                    continueSearch = true;
//                else if (tt.mCurrentStrategy == TraverserStrategy::BOTTOM_TO_TOP)
//                    continueSearch = false;
            }
        }

        void goStepDown(ChildrenNode<T, L>* childrenNode)
        {
            if (tt.mFilter && tt.mFilter->filter(childrenNode->getChild(tt.mCurrentChildId)))
            {
                ++tt.mCurrentChildId;
                continueSearch = true;
            }
            else
            {
                //std::cout << "down.\n";
                tt.goStepDown(childrenNode);
                if (tt.mCurrentStrategy == TraverserStrategy::TOP_TO_BOTTOM)
                    continueSearch = false;
                else if (tt.mCurrentStrategy == TraverserStrategy::BOTTOM_TO_TOP)
                    continueSearch = true;
            }
        }

        TreeTraverser<T, L>& tt;
        bool continueSearch;
        bool initialStep;
    };

    TreeVisitor visitor(*this);
    mCurrentNode->accept(visitor);
    while (visitor.continueSearch)
    {
        mCurrentNode->accept(visitor);
        //std::cout << ".......................\n";
    }
    return mCurrentNode;
}

// TODO: this is unused?
// TreeTraverser doesn't has to inherit from NodeVisitor
template <class T, class L>
void TreeTraverser<T, L>::visit(ChildrenNode<T, L>* childrenNode)
{
    // as long as there are elements left in the stack go up
    // until a node is reached that has children that were not
    // yet iterated left

    // As long no child left to go down to
    // go up or
    // if going up not possible (at root) set NULL
    while (mCurrentChildId >= childrenNode->getNumberOfChildren())
    {

        if (childrenNode->getParent() == NULL || mStack.empty())
        {
            mCurrentNode = NULL;
            break;
        }
        else
        {
            goStepUp();
            childrenNode = static_cast<ChildrenNode<T, L>*>(mCurrentNode);
        }
    }
    // If there is a child left go down
    if (mCurrentNode != NULL)
    {
        goStepDown(childrenNode);
    }
}

template <class T, class L>
void TreeTraverser<T, L>::visit(LeafNode<T, L>* /*leafNode*/)
{
    // no children so go up
    goStepUp();
}

template <class T, class L>
void TreeTraverser<T, L>::goStepDown(ChildrenNode<T, L>* childrenNode)
{
    // push current state to the stack
    mStack.push(mCurrentChildId);
    // go down to next child
    mCurrentNode = childrenNode->getChild(mCurrentChildId);
    mCurrentChildId = 0;
}

template <class T, class L>
void TreeTraverser<T, L>::goStepUp()
{
    // go up
    // go to parent
    mCurrentNode = mCurrentNode->getParent();

    // child id + 1 of parent
    mCurrentChildId = mStack.top();
    ++mCurrentChildId;
    mStack.pop();
}

template <class T, class L>
TraversalFilter<T, L>* TreeTraverser<T, L>::getFilter() const
{
    return mFilter;
}

template <class T, class L>
void TreeTraverser<T, L>::setFilter(TraversalFilter<T, L>* filter)
{
    mFilter = filter;
}

template <class T, class L>
size_t TreeTraverser<T, L>::getCurrentLevel() const
{
    return mStack.size();
}

// Tree Iterator

template <class T, class L>
TreeTraverser<T, L>::Iterator::Iterator(Node<T, L>* currentNode, int childCount)
    : mChildCount(childCount)
    , mCurrentNode(currentNode)
{

}

template <class T, class L>
int TreeTraverser<T, L>::Iterator::getChildCount() const
{
    return mChildCount;
}

template <class T, class L>
Node<T, L>* TreeTraverser<T, L>::Iterator::getCurrentNode() const
{
    return mCurrentNode;
}

template <class T, class L>
bool TreeTraverser<T, L>::Iterator::operator==(TreeTraverser<T, L>::Iterator const& it) const
{
    return mChildCount == it.getChildCount() &&
            mCurrentNode == it.getCurrentNode();
}

template <class T, class L>
Node<T, L>* TreeTraverser<T, L>::Iterator::operator->()
{
    return mCurrentNode;
}

template <class T, class L>
Node<T, L> const* TreeTraverser<T, L>::Iterator::operator->() const
{
    return mCurrentNode;
}

template <class T, class L>
Node<T, L>& TreeTraverser<T, L>::Iterator::operator*()
{
    return *mCurrentNode;
}

template <class T, class L>
Node<T, L> TreeTraverser<T, L>::Iterator::operator*() const
{
    return *mCurrentNode;
}


#endif // TREETRAVERSER_CPP


