#pragma once

#ifndef SYNCEDMEMBERACCESSOR_CPP
#define SYNCEDMEMBERACCESSOR_CPP

#include "SyncedMemberAccessor.h"

#include <multi_threading/Domain.h>
#include <multi_threading/Operation.h>

template<class T>
SyncedMemberAccessor<T>::SyncedMemberAccessor(
        Domain* domain,
        std::shared_ptr<MemberAccessor<T>> memberAccessor)
    : mDomain(domain)
    , mMemberAccessor(memberAccessor)
{
}

template<class T>
SyncedMemberAccessor<T>::~SyncedMemberAccessor()
{

}

template<class T>
void* SyncedMemberAccessor<T>::getOwner()
{
    return mMemberAccessor->getOwner();
}

template<class T>
void SyncedMemberAccessor<T>::setOwner(void* owner)
{
    // Commented out for now because setting the owner is usually not a
    // critical operation since it doesn't affect the data that is accessed.
//    if (mDomain)
//    {
//        // create an operation and add it to the operation queue
//        std::function<void()> f = [this, owner](){mMemberAccessor->setOwner(owner);};
//        mDomain->addOperation(new Operation(f));
//    }
//    else
//    {
        mMemberAccessor->setOwner(owner);
//    }
}

template<class T>
T SyncedMemberAccessor<T>::getData()
{
    return mMemberAccessor->getData();
}

template<class T>
void SyncedMemberAccessor<T>::setData(T data)
{
    if (mDomain)
    {
        // create an operation and add it to the operation queue
        std::function<void()> f = [this, data](){mMemberAccessor->setData(data);};
        mDomain->addOperation(new Operation(f));
    }
    else
    {
        mMemberAccessor->setData(data);
    }
}

#endif // SYNCEDMEMBERACCESSOR_CPP
