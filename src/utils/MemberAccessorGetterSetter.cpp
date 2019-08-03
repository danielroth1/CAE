#pragma once

#ifndef MEMBERACCESSORGETTERSETTER_CPP
#define MEMBERACCESSORGETTERSETTER_CPP

#include "MemberAccessorGetterSetter.h"


template<class T, class OwnerType>
MemberAccessorGetterSetter<T, OwnerType>::MemberAccessorGetterSetter(
        std::function<T (OwnerType*)> getter,
        std::function<void (OwnerType*, T)> setter,
        T defaultValue,
        OwnerType* owner)
    : OwnerMemberAccessor<T> (defaultValue, owner)
    , mGetter(getter)
    , mSetter(setter)
{
}

template<class T, class ObjType>
MemberAccessorGetterSetter<T, ObjType>::~MemberAccessorGetterSetter()
{

}

template<class T, class OwnerType>
T MemberAccessorGetterSetter<T, OwnerType>::getData(size_t ownerIndex)
{
    return mGetter(getOwnerWithType(ownerIndex));
}

template<class T, class OwnerType>
void MemberAccessorGetterSetter<T, OwnerType>::setData(T data, size_t ownerIndex)
{
    mSetter(getOwnerWithType(ownerIndex), data);
}

template<class T, class OwnerType>
void MemberAccessorGetterSetter<T, OwnerType>::addOwnerWithType(OwnerType* owner)
{
    this->addOwner(static_cast<OwnerType*>(owner));
}

template<class T, class OwnerType>
OwnerType* MemberAccessorGetterSetter<T, OwnerType>::getOwnerWithType(
        size_t ownerIndex)
{
    return static_cast<OwnerType*>(this->getOwners()[ownerIndex]);
}

#endif // MEMBERACCESSORGETTERSETTER_CPP
