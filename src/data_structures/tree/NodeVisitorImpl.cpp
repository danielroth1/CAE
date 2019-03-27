#ifndef NODEVISITORIMPL_CPP
#define NODEVISITORIMPL_CPP

#include "NodeVisitorImpl.h"

#include "ChildrenNode.h"
#include "LeafNode.h"

template <class T, class L>
NodeVisitorImpl<T, L>::NodeVisitorImpl()
{

}

template <class T, class L>
NodeVisitorImpl<T, L>::~NodeVisitorImpl()
{

}

template <class T, class L>
void NodeVisitorImpl<T, L>::visit(ChildrenNode<T, L>* /*childrenNode*/)
{

}

template <class T, class L>
void NodeVisitorImpl<T, L>::visit(LeafNode<T, L>* /*leafNode*/)
{

}

#endif // NODEVISITORIMPL_CPP
