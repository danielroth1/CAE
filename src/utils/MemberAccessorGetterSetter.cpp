#pragma once

#ifndef MEMBERACCESSORGETTERSETTER_CPP
#define MEMBERACCESSORGETTERSETTER_CPP

#include "MemberAccessorGetterSetter.h"


template<class T, class ObjType>
MemberAccessorGetterSetter<T, ObjType>::MemberAccessorGetterSetter(
        std::function<T (ObjType*)> getter,
        std::function<void (ObjType*, T)> setter,
        ObjType* object)
    : mGetter(getter)
    , mSetter(setter)
    , mObject(object)
{

}

template<class T, class ObjType>
MemberAccessorGetterSetter<T, ObjType>::~MemberAccessorGetterSetter()
{

}

template<class T, class ObjType>
void MemberAccessorGetterSetter<T, ObjType>::setObject(ObjType* object)
{
    mObject = object;
}

template<class T, class ObjType>
ObjType* MemberAccessorGetterSetter<T, ObjType>::getObject()
{
    return mObject;
}

template<class T, class ObjType>
T MemberAccessorGetterSetter<T, ObjType>::getData()
{
    return mGetter(mObject);
}

template<class T, class ObjType>
void MemberAccessorGetterSetter<T, ObjType>::setData(T data)
{
    if (mObject)
        mSetter(mObject, data);
}

#endif // MEMBERACCESSORGETTERSETTER_CPP
