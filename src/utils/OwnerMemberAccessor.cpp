#pragma once

#ifndef OWNERMEMBERACCESSOR_CPP
#define OWNERMEMBERACCESSOR_CPP

#include "OwnerMemberAccessor.h"

template<class T>
OwnerMemberAccessor<T>::OwnerMemberAccessor(T defaultValue, void* owner)
    : mDefaultValue(defaultValue)
{
    mOwner = owner;
}

template<class T>
OwnerMemberAccessor<T>::~OwnerMemberAccessor()
{

}

template<class T>
bool OwnerMemberAccessor<T>::hasOwner() const
{
    return mOwner != nullptr;
}

template<class T>
void* OwnerMemberAccessor<T>::getOwner()
{
    return mOwner;
}

template<class T>
void OwnerMemberAccessor<T>::setOwner(void* owner)
{
    mOwner = owner;
}

template<class T>
T OwnerMemberAccessor<T>::getDefaultValue()
{
    return mDefaultValue;
}

template<class T>
void OwnerMemberAccessor<T>::setDefaultValue(T defaultValue)
{
    mDefaultValue = defaultValue;
}

//template<class T>
//void* OwnerMemberAccessor<T>::getOwner()
//{
//    return mOwner;
//}

//template<class T>
//void OwnerMemberAccessor<T>::setOwner(void* owner)
//{
//    mOwner = owner;
//}

#endif // OWNERMEMBERACCESSOR_CPP
