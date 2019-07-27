#pragma once

#ifndef MEMBERACCESSOR_CPP
#define MEMBERACCESSOR_CPP

#include "MemberAccessor.h"

template<class T>
MemberAccessor<T>::MemberAccessor()
{

}

template<class T>
MemberAccessor<T>::~MemberAccessor()
{

}

template<class T>
bool MemberAccessor<T>::hasOwner() const
{
    return false;
}

template<class T>
void* MemberAccessor<T>::getOwner()
{
    return nullptr;
}

template<class T>
void MemberAccessor<T>::setOwner(void* /*owner*/)
{
    // do nothing
}

#endif // MEMBERACCESSOR_CPP
