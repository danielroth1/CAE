#pragma once

#ifndef MEMBERACCESSORINTERFACE_CPP
#define MEMBERACCESSORINTERFACE_CPP

#include "MemberAccessorInterface.h"

template<class T>
MemberAccessorInterface<T>::MemberAccessorInterface()
{

}

template<class T>
MemberAccessorInterface<T>::~MemberAccessorInterface()
{

}

template<class T>
bool MemberAccessorInterface<T>::operator==(MemberAccessorInterface<T>& a)
{
    return getData() == a.getData();
}

#endif // MEMBERACCESSORINTERFACE_CPP
