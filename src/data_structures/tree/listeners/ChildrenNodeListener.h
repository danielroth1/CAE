#ifndef CHILDRENNODELISTENER_H
#define CHILDRENNODELISTENER_H

#include "NodeListener.h"

template <class T, class L>
class Node;

template <class T, class L>
class ChildrenNodeListener
{
public:
    virtual void notifyChildAdded(Node<T, L>* source, Node<T,L>* childNode) = 0;
    virtual void notifyChildRemoved(Node<T, L>* source, Node<T,L>* childNode) = 0;
    virtual void notifyChildrenDataChanged(Node<T, L>* source, T& data) = 0;

protected:
    ChildrenNodeListener();
    virtual ~ChildrenNodeListener();
};

#include "ChildrenNodeListener.cpp"

#endif // CHILDRENNODELISTENER_H
