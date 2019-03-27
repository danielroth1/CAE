#ifndef POINTREF_CPP
#define POINTREF_CPP

#include "PointRef.h"

template <class T, class P>
PointRef<T, P>::PointRef(T object, ID pointIndex)
    : mObject(object)
    , mIndex(pointIndex)
{

}

template<class T, class P>
T PointRef<T, P>::getObject()
{
    return mObject;
}

template<class T, class P>
ID PointRef<T, P>::getPointIndex()
{
    return mIndex;
}

#endif // POINTREF_CPP
