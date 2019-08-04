#pragma once

#ifndef MEMBERACCESSORDIRECT_CPP
#define MEMBERACCESSORDIRECT_CPP

#include "MemberAccessorDirect.h"

template<class T>
MemberAccessorDirect<T>::MemberAccessorDirect(
        T* data,
        const std::shared_ptr<std::function<bool(T, T)>>& comparator)
    : MemberAccessor<T> (comparator)
    , mDataPtr(data)
{

}

template<class T>
MemberAccessorDirect<T>::~MemberAccessorDirect()
{

}

template<class T>
T MemberAccessorDirect<T>::getData()
{
    return *mDataPtr;
}

template<class T>
void MemberAccessorDirect<T>::setData(T data)
{
    &mDataPtr = data;
}

#endif // MEMBERACCESSORDIRECT_CPP
