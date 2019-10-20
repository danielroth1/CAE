#ifndef BIDIRECTIONALMAP_CPP
#define BIDIRECTIONALMAP_CPP

#include "BidirectionalMap.h"

template<class A, class B>
BidirectionalMap<A, B>::BidirectionalMap()
{

}

template<class A, class B>
void BidirectionalMap<A, B>::clear()
{
    mAToBMap.clear();
    mBToAMap.clear();
}

template<class A, class B>
void BidirectionalMap<A, B>::add(A a, B b)
{
    mAToBMap[a] = b;
    mBToAMap[b] = a;
}

template<class A, class B>
A BidirectionalMap<A, B>::get(B b)
{
    return mBToAMap[b];
}

template<class A, class B>
B BidirectionalMap<A, B>::get(A a)
{
    return mAToBMap[a];
}

template<class A, class B>
typename std::map<A, B>::iterator BidirectionalMap<A, B>::find(A a)
{
    return mAToBMap.find(a);
}

template<class A, class B>
typename std::map<A, B>::iterator BidirectionalMap<A, B>::find(B b)
{
    return mBToAMap.find(b);
}

template<class A, class B>
typename std::map<A, B>::iterator BidirectionalMap<A, B>::end(A)
{
    return mAToBMap.end();
}

template<class A, class B>
typename std::map<A, B>::iterator BidirectionalMap<A, B>::end(B)
{
    return mBToAMap.end();
}

template<class A, class B>
bool BidirectionalMap<A, B>::remove(A a)
{
    auto it = mAToBMap.find(a);
    if (it != mAToBMap.end())
    {
        mAToBMap.erase(it);
        mBToAMap.erase(mBToAMap.find(it->second));
        return true;
    }
    return false;
}

template<class A, class B>
bool BidirectionalMap<A, B>::remove(B b)
{
    auto it = mBToAMap.find(b);
    if (it != mBToAMap.end())
    {
        mBToAMap.erase(it);
        mAToBMap.erase(mAToBMap.find(it->second));
        return true;
    }
    return false;
}

#endif // BIDIRECTIONALMAP_CPP
