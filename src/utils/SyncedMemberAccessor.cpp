#pragma once

#ifndef SYNCEDMEMBERACCESSOR_CPP
#define SYNCEDMEMBERACCESSOR_CPP

#include "SyncedMemberAccessor.h"

#include <multi_threading/Domain.h>
#include <multi_threading/Operation.h>

template<class T>
SyncedMemberAccessor<T>::SyncedMemberAccessor(
        Domain* domain,
        const std::shared_ptr<MemberAccessorInterface<T>>& memberAccessor)
    : mDomain(domain)
    , mMemberAccessor(memberAccessor)
{
}

template<class T>
SyncedMemberAccessor<T>::~SyncedMemberAccessor()
{

}

template<class T>
bool SyncedMemberAccessor<T>::operator==(MemberAccessorInterface<T>& a)
{
    return *mMemberAccessor.get() == a;
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

template<class T>
MemberAccessorType SyncedMemberAccessor<T>::getType() const
{
    return mMemberAccessor->getType();
}

#endif // SYNCEDMEMBERACCESSOR_CPP
