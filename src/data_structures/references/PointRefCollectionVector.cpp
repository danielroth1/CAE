#ifndef POINTREFCOLLECTIONVECTOR_CPP
#define POINTREFCOLLECTIONVECTOR_CPP

#include "PointRefCollectionVector.h"

template<class T, class P>
PointRefCollectionVector<T, P>::PointRefCollectionVector(T object)
    : PointRefCollection<T, P>()
    , mObject(object)
{

}

template<class T, class P>
void PointRefCollectionVector<T, P>::addPoint(ID index)
{
    mIndices.push_back(index);
}

template<class T, class P>
void PointRefCollectionVector<T, P>::addPoints(const std::vector<ID>& indices)
{
    mIndices.resize(indices.size());
    std::copy(indices.begin(), indices.end(), mIndices.begin());
}

template<class T, class P>
bool PointRefCollectionVector<T, P>::removePoint(ID index)
{
    // TODO: use lambda expression to iterate over vector
//    auto it = std::find(mIndices.begin(), mIndices.end(),
//                        [index]{return });
    for (auto it = mIndices.begin(); it != mIndices.end(); ++it)
    {
        if (*it == index)
        {
            mIndices.erase(it);
            return true;
        }
    }
    return false;
}

template<class T, class P>
void PointRefCollectionVector<T, P>::removePoints()
{
    mIndices.clear();
}

//template<class T, class P>
//P PointRefCollectionVector<T, P>::getPoint(ID index)
//{
//    return mVector[index];
//}

template<class T, class P>
ID PointRefCollectionVector<T, P>::getSize()
{
    return mIndices.size();
}

template<class T, class P>
ID PointRefCollectionVector<T, P>::getPointIndex(ID index)
{
    return mIndices[index];
}

template<class T, class P>
T PointRefCollectionVector<T, P>::getObject()
{
    return mObject;
}

template<class T, class P>
std::vector<ID>::iterator
PointRefCollectionVector<T, P>::begin()
{
    return mIndices.begin();
}

template<class T, class P>
std::vector<ID>::iterator
PointRefCollectionVector<T, P>::end()
{
    return mIndices.end();
}

#endif // POINTREFCOLLECTIONVECTOR_CPP
