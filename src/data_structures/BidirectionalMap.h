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

    typename std::map<A, B>::iterator find(A a);
    typename std::map<B, A>::iterator find(B b);

    typename std::map<A, B>::iterator end(A /*a*/);
    typename std::map<B, A>::iterator end(B /*b*/);

    bool remove(A a);
    bool remove(B b);

    const std::map<A, B>& getFirstMap() const;
    const std::map<B, A>& getSecondMap() const;

private:
    std::map<A, B> mAToBMap;
    std::map<B, A> mBToAMap;
};

#include "BidirectionalMap.cpp"

#endif // BIDIRECTIONALMAP_H
