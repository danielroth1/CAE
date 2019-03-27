#ifndef NODEVISITORIMPL_H
#define NODEVISITORIMPL_H

//#include "ChildrenNode.h"
#include "NodeVisitor.h"
//#include "LeafNode.h"

template <class T, class L>
class ChildrenNode;

template <class T, class L>
class LeafNode;

template <class T, class L>
class NodeVisitorImpl : public NodeVisitor<T, L>
{
public:
    NodeVisitorImpl();
    virtual ~NodeVisitorImpl();

    virtual void visit(ChildrenNode<T, L>* childrenNode);
    virtual void visit(LeafNode<T, L>* leafNode);
};

#include "NodeVisitorImpl.cpp"

#endif // NODEVISITORIMPL_H
