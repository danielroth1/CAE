#ifndef TRAVERSALFILTER_H
#define TRAVERSALFILTER_H

template <class T, class L>
class Node;

template <class T, class L>
class TraversalFilter
{
public:
    TraversalFilter();
    virtual ~TraversalFilter();

    virtual bool filter(Node<T, L>* data) = 0;
};

#include "TraversalFilter.cpp"

#endif // TRAVERSALFILTER_H
