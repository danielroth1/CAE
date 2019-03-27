#ifndef LEAFNODELISTENER_H
#define LEAFNODELISTENER_H

#include "NodeListener.h"


template <class T, class L>
class LeafNodeListener
{
public:
    virtual void notifyLeafDataChanged(Node<T, L>* source, L& data) = 0;

protected:
    LeafNodeListener();
    virtual ~LeafNodeListener();
};

#include "LeafNodeListener.cpp"

#endif // LEAFNODELISTENER_H
