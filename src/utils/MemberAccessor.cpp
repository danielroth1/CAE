#pragma once

#ifndef MEMBERACCESSOR_CPP
#define MEMBERACCESSOR_CPP

#include "MemberAccessor.h"


template<class T>
MemberAccessor<T>::MemberAccessor(
        const std::shared_ptr<std::function<bool(T, T)>>& equalityComparator)
    : mEqualityComparator(equalityComparator)
{

}

template<class T>
MemberAccessor<T>::~MemberAccessor()
{

}

template<class T>
bool MemberAccessor<T>::operator==(MemberAccessorInterface<T>& a)
{
    if (mEqualityComparator)
        return (*mEqualityComparator)(getData(), a.getData());
    else
        return getData() == a.getData();
}

template<class T>
MemberAccessorType MemberAccessor<T>::getType() const
{
    return MemberAccessorType::MEMBER_ACCESSOR_INTERFACE;
}

#endif // MEMBERACCESSOR_CPP
