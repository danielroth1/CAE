#pragma once

#ifndef OWNERMEMBERACCESSOR_CPP
#define OWNERMEMBERACCESSOR_CPP

#include "OwnerMemberAccessor.h"
#include <algorithm>


template<class T>
OwnerMemberAccessor<T>::OwnerMemberAccessor(
        T defaultValue,
        void* owner,
        const std::shared_ptr<std::function<bool(T, T)>>& comparator)
    : MemberAccessor<T> (comparator)
    , mDefaultValue(defaultValue)
{
    if (owner)
        addOwner(owner);
}

template<class T>
OwnerMemberAccessor<T>::~OwnerMemberAccessor()
{

}

template<class T>
T OwnerMemberAccessor<T>::getData()
{
    // TODO: use a comparator and check if the data is identical and if
    // it is, return that value instead of the default value.

    // if there is no owner or more than one, return default value
    if (this->getOwners().empty())
    {
        return this->getDefaultValue();
    }
    return this->getData(0);
}

template<class T>
void OwnerMemberAccessor<T>::setData(T data)
{
    for (size_t i = 0; i < OwnerMemberAccessor<T>::mOwners.size(); ++i)
    {
        setData(data, i);
    }
}

template<class T>
MemberAccessorType OwnerMemberAccessor<T>::getType() const
{
    return MemberAccessorType::OWNER_MEMBER_ACCESSOR_INTERFACE;
}

template<class T>
size_t OwnerMemberAccessor<T>::getNumOwners() const
{
    return mOwners.size();
}

template<class T>
const std::vector<void*>& OwnerMemberAccessor<T>::getOwners() const
{
    return mOwners;
}

template<class T>
void OwnerMemberAccessor<T>::setOwners(const std::vector<void*>& owners)
{
    mOwners = owners;
}

template<class T>
void OwnerMemberAccessor<T>::addOwner(void* owner)
{
    auto it = std::find(mOwners.begin(), mOwners.end(), owner);
    if (it == mOwners.end())
        mOwners.push_back(owner);
}

template<class T>
void OwnerMemberAccessor<T>::removeOwner(void* owner)
{
    auto it = std::find(mOwners.begin(), mOwners.end(), owner);
    if (it != mOwners.end())
    {
        mOwners.erase(it);
    }
}

template<class T>
void OwnerMemberAccessor<T>::clearOwners()
{
    mOwners.clear();
}

template<class T>
void OwnerMemberAccessor<T>::setData(const std::vector<T>& data)
{
    for (size_t i = 0; i < mOwners.size(); ++i)
    {
        setData(data[i], i);
    }
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

template<class T>
bool OwnerMemberAccessor<T>::isAccessorValuesIdentical()
{
    for (size_t i = 1; i < mOwners.size(); ++i)
    {
        if (this->getData(i-1) != this->getData(i))
        {
            return false;
        }
    }
    return true;
}

#endif // OWNERMEMBERACCESSOR_CPP
