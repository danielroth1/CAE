#ifndef POINTREFCOLLECTIONMAP_CPP
#define POINTREFCOLLECTIONMAP_CPP

#include "PointRefCollectionMap.h"

template <class T, class P>
PointRefCollectionMap<T, P>::PointRefCollectionMap()
{

}

template<class T, class P>
void PointRefCollectionMap<T, P>::addPoint(T object, ID index)
{
    auto it = mMap.find(object);
    if (it != mMap.end())
    {
        it->push_back(index);
    }
    else
    {
        std::vector<ID> vs;
        vs.push_back(index);
        mMap[object] = vs;
    }
}

template<class T, class P>
void PointRefCollectionMap<T, P>::addPoints(T object, const std::vector<ID>& indices)
{
    for (ID index : indices)
    {
        addPoint(object, index);
    }
}

template<class T, class P>
bool PointRefCollectionMap<T, P>::removePoint(T object, ID index)
{
    auto it = mMap.find(object);
    if (it != mMap.end())
    {
        auto it2 = mMap.find(index);
        if (it2 != it->end())
        {
            it->erase(it2);
            return true;
        }
        else
        {
            // there is no index to remove
            return false;
        }
    }
    else
    {
        // there is no object
        return false;
    }
}

template<class T, class P>
bool PointRefCollectionMap<T, P>::removePoints(T object)
{
    auto it = mMap.find(object);
    if (it != mMap.end())
    {
        mMap.erase(it);
    }
    else
    {
        // there is no object
        return false;
    }
}

template<class T, class P>
ID PointRefCollectionMap<T, P>::getSize()
{
    return mMap.size();
}

template<class T, class P>
ID PointRefCollectionMap<T, P>::getPointIndex(T object, ID index)
{
    return mMap[object][index];
}

template<class T, class P>
std::vector<ID> PointRefCollectionMap<T, P>::getPointIndices(T object)
{
    return mMap[object];
}

template<class T, class P>
typename std::map<T, std::vector<ID>>::iterator
PointRefCollectionMap<T, P>::begin()
{
    return mMap.begin();
}

template<class T, class P>
typename std::map<T, std::vector<ID>>::iterator
PointRefCollectionMap<T, P>::end()
{
    return mMap.end();
}

#endif // POINTREFCOLLECTIONMAP_CPP
