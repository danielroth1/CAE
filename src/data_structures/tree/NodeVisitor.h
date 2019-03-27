#ifndef NODEVISITOR_H
#define NODEVISITOR_H

template <class T, class L>
class ChildrenNode;

template <class T, class L>
class LeafNode;

//#include "ChildrenNode.h"
//#include "LeafNode.h"

template <class T, class L>
class NodeVisitor
{
public:

    virtual void visit(ChildrenNode<T, L>* childrenNode) = 0;
    virtual void visit(LeafNode<T, L>* leafNode) = 0;

    NodeVisitor() {}
    virtual ~NodeVisitor() {}

};

#endif // NODEVISITOR_H
