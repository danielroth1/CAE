#ifndef TREELISTENER_H
#define TREELISTENER_H

#include "NodeListener.h"
#include "ChildrenNodeListener.h"
#include "LeafNodeListener.h"


template <class T, class L>
class TreeListener : public NodeListener<T, L>,
        public ChildrenNodeListener<T, L>,
        public LeafNodeListener<T, L>
{
public:
    TreeListener();
    virtual ~TreeListener();
};

#include "TreeListener.cpp"

#endif // TREELISTENER_H
