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
T* NodeData<T, L>::getChildData(unsigned int i) const
{
    mGetChildVisitor.childIndex = i;
    Node<T, L>* node = mGetChildVisitor->visit(mNode);
    mGetDataVisitor->visit(node);
    return mGetDataVisitor.returnValue;
}

template<class T, class L>
std::size_t NodeData<T, L>::getNumberOfChildren() const
{
    mGetNumberOfChildrenVisitor->visit(mNode);
    return mGetNumberOfChildrenVisitor.returnValue;
}

#endif // NODEDATA_CPP
