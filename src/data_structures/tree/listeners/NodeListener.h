#ifndef NODELISTENER_H
#define NODELISTENER_H

#include <string>
#include <vector>

template <class T, class L>
class Node;

template <class T, class L>
class Tree;

template <class T, class L>
class NodeListener
{
public:
    virtual void notifyParentChanged(Node<T, L>* source, Node<T, L>* parent) = 0;
    virtual void notifyNameChanged(Node<T, L>* source, std::string name) = 0;
    virtual void notifyTreeChanged(Node<T, L>* source, Tree<T, L>* tree) = 0;

    NodeListener();
    virtual ~NodeListener();
};

#include "NodeListener.cpp"

#endif // NODELISTENER_H
