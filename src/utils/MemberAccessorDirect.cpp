#pragma once

#ifndef MEMBERACCESSORDIRECT_CPP
#define MEMBERACCESSORDIRECT_CPP

#include "MemberAccessorDirect.h"

template<class T>
MemberAccessorDirect<T>::MemberAccessorDirect(T* data)
    : mDataPtr(data)
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
