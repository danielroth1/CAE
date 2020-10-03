#ifndef NODEDATA_CPP
#define NODEDATA_CPP

#include "NodeData.h"

template<class T, class L>
NodeData<T, L>::NodeData(Node<T, L>* node)
    : mNode(node)
{

}

template<class T, class L>
Node<T, L>* NodeData<T, L>::getNode()
{
    return mNode;
}

template<class T, class L>
std::size_t NodeData<T, L>::getNumberOfChildren() const
{
    mGetNumberOfChildrenVisitor->visit(mNode);
    return mGetNumberOfChildrenVisitor.returnValue;
}

#endif // NODEDATA_CPP
