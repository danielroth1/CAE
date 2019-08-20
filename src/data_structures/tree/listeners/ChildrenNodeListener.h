#ifndef CHILDRENNODELISTENER_H
#define CHILDRENNODELISTENER_H

#include "NodeListener.h"

template <class T, class L>
class Node;

template <class T, class L>
class ChildrenNodeListener
{
public:
    // \param source - parent
    // \paran childNore - child
    virtual void notifyChildAdded(Node<T, L>* source, Node<T,L>* childNode) = 0;

    // \param source - parent
    // \paran childNore - child
    virtual void notifyChildRemoved(Node<T, L>* source, Node<T,L>* childNode) = 0;

    // \param source - node whose data changed
    // \param data - data of that node
    virtual void notifyChildrenDataChanged(Node<T, L>* source, T& data) = 0;

protected:
    ChildrenNodeListener();
    virtual ~ChildrenNodeListener();
};

#include "ChildrenNodeListener.cpp"

#endif // CHILDRENNODELISTENER_H
