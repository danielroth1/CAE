#ifndef BIDIRECTIONALMAP_H
#define BIDIRECTIONALMAP_H


#include <map>

template<class A, class B>
class BidirectionalMap
{
public:
    BidirectionalMap();

    void clear();

    void add(A a, B b);

    A get(B b);
    B get(A a);

    bool remove(A a);
    bool remove(B b);

private:
    std::map<A, B> mAToBMap;
    std::map<B, A> mBToAMap;
};

#include "BidirectionalMap.cpp"

#endif // BIDIRECTIONALMAP_H
