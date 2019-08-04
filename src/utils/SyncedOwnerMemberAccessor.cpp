#pragma once

#ifndef SYNCEDOWNERMEMBERACCESSOR_CPP
#define SYNCEDOWNERMEMBERACCESSOR_CPP

#include "SyncedOwnerMemberAccessor.h"

#include <multi_threading/Domain.h>
#include <multi_threading/Operation.h>


template<class T>
SyncedOwnerMemberAccessor<T>::SyncedOwnerMemberAccessor(
        Domain* domain,
        const std::shared_ptr<OwnerMemberAccessorInterface<T>>& accessor)
    : mDomain(domain)
    , mAccessor(accessor)
{
}

template<class T>
T SyncedOwnerMemberAccessor<T>::getData()
{
    return mAccessor->getData();
}

template<class T>
void SyncedOwnerMemberAccessor<T>::setData(T data)
{
    // Commented out for now because setting the owner is usually not a
    // critical operation since it doesn't affect the data that is accessed.

    if (mDomain)
    {
        std::function<void()> f = [this, data]()
        {
            mAccessor->setData(data);
        };
        mDomain->addOperation(new Operation(f));
    }
    else
    {
        mAccessor->setData(data);
    }
}

template<class T>
MemberAccessorType SyncedOwnerMemberAccessor<T>::getType() const
{
    return mAccessor->getType();
}

template<class T>
T SyncedOwnerMemberAccessor<T>::getData(size_t ownerIndex)
{
    return mAccessor->getData(ownerIndex);
}

template<class T>
void SyncedOwnerMemberAccessor<T>::setData(T data, size_t ownerIndex)
{
    mAccessor->setData(data, ownerIndex);
}

template<class T>
size_t SyncedOwnerMemberAccessor<T>::getNumOwners() const
{
    return mAccessor->getNumOwners();
}

template<class T>
const std::vector<void*>& SyncedOwnerMemberAccessor<T>::getOwners() const
{
    return mAccessor->getOwners();
}

template<class T>
void SyncedOwnerMemberAccessor<T>::setOwners(const std::vector<void*>& owners)
{
    if (mDomain)
    {
        std::function<void()> f = [this, owners]()
        {
            mAccessor->setOwners(owners);
        };
        mDomain->addOperation(new Operation(f));
    }
    else
    {
        mAccessor->setOwners(owners);
    }
}

template<class T>
void SyncedOwnerMemberAccessor<T>::addOwner(void* owner)
{
//    if (mDomain)
//    {
//        std::function<void()> f = [this, owner]()
//        {
//            mAccessor->addOwner(owner);
//        };
//        mDomain->addOperation(new Operation(f));
//    }
//    else
//    {
        mAccessor->addOwner(owner);
//    }
}

template<class T>
void SyncedOwnerMemberAccessor<T>::removeOwner(void* owner)
{
//    if (mDomain)
//    {
//        std::function<void()> f = [this, owner]()
//        {
//            mAccessor->removeOwner(owner);
//        };
//        mDomain->addOperation(new Operation(f));
//    }
//    else
//    {
        mAccessor->removeOwner(owner);
        //    }
}

template<class T>
void SyncedOwnerMemberAccessor<T>::clearOwners()
{
    mAccessor->clearOwners();
}

template<class T>
void SyncedOwnerMemberAccessor<T>::setData(const std::vector<T>& data)
{
    if (mDomain)
    {
        std::function<void()> f = [this, data]()
        {
            mAccessor->setData(data);
        };
        mDomain->addOperation(new Operation(f));
    }
    else
    {
        mAccessor->setData(data);
    }
}

template<class T>
T SyncedOwnerMemberAccessor<T>::getDefaultValue()
{
    return mAccessor->getDefaultValue();
}

template<class T>
void SyncedOwnerMemberAccessor<T>::setDefaultValue(T defaultValue)
{
    if (mDomain)
    {
        std::function<void()> f = [this, defaultValue]()
        {
            mAccessor->setDefaultValue(defaultValue);
        };
        mDomain->addOperation(new Operation(f));
    }
    else
    {
        mAccessor->setDefaultValue(defaultValue);
    }
}

template<class T>
bool SyncedOwnerMemberAccessor<T>::isAccessorValuesIdentical()
{
    return mAccessor->isAccessorValuesIdentical();
}



#endif // SYNCEDOWNERMEMBERACCESSOR_H
