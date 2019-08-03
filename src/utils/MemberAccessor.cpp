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
bool MemberAccessor<T>::operator==(const MemberAccessor<T>& a)
{
    return getData() == a.getData();
}

template<class T>
MemberAccessorType MemberAccessor<T>::getType() const
{
    return MemberAccessorType::MEMBER_ACCESSOR;
}

#endif // MEMBERACCESSOR_CPP
