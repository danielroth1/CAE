#pragma once

#ifndef MEMBERACCESSORGETTER_CPP
#define MEMBERACCESSORGETTER_CPP

#include "MemberAccessorGetter.h"

template<class T, class ObjType>
MemberAccessorGetter<T, ObjType>::MemberAccessorGetter(
        std::function<T&(ObjType*)> getterRef,
        ObjType* object,
        const std::shared_ptr<std::function<bool(T, T)>>& comparator)
    : MemberAccessor<T> (comparator)
    , mGetterRef(getterRef)
    , mObject(object)
{

}

template<class T, class ObjType>
MemberAccessorGetter<T, ObjType>::~MemberAccessorGetter()
{

}

template<class T, class ObjType>
void MemberAccessorGetter<T, ObjType>::setObject(ObjType* object)
{
    mObject = object;
}

template<class T, class ObjType>
ObjType* MemberAccessorGetter<T, ObjType>::getObject()
{
    return mObject;
}

template<class T, class ObjType>
T MemberAccessorGetter<T, ObjType>::getData()
{
    return mGetterRef(mObject);
}

template<class T, class ObjType>
void MemberAccessorGetter<T, ObjType>::setData(T data)
{
    if (mObject)
        mGetterRef(mObject) = data;
}

#endif // MEMBERACCESSORGETTER_CPP
