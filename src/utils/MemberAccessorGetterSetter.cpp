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
T MemberAccessorGetterSetter<T, OwnerType>::getData()
{
    if (!this->getOwner())
        return this->getDefaultValue();
    return mGetter(getOwnerWithType());
}

template<class T, class ObjType>
void MemberAccessorGetterSetter<T, ObjType>::setData(T data)
{
    if (this->getOwner())
        mSetter(getOwnerWithType(), data);
}

template<class T, class OwnerType>
void MemberAccessorGetterSetter<T, OwnerType>::setOwnerWithType(OwnerType* owner)
{
    this->setOwner(owner);
}

template<class T, class OwnerType>
OwnerType* MemberAccessorGetterSetter<T, OwnerType>::getOwnerWithType()
{
    return static_cast<OwnerType*>(this->getOwner());
}

#endif // MEMBERACCESSORGETTERSETTER_CPP
