#pragma once

#ifndef MEMBERACCESSORFACTORY_CPP
#define MEMBERACCESSORFACTORY_CPP

#include "MemberAccessorDirect.h"
#include "MemberAccessorFactory.h"
#include "MemberAccessorGetter.h"
#include "MemberAccessorGetterSetter.h"
#include "SyncedMemberAccessor.h"
#include "SyncedOwnerMemberAccessor.h"


#endif // MEMBERACCESSORFACTORY_H

template<class T>
std::shared_ptr<MemberAccessor<T>>
MemberAccessorFactory::createDirect(
        T* data,
        Domain* domain)
{
    std::shared_ptr<MemberAccessor<T>> accessor =
            std::make_shared<MemberAccessorDirect<T>>(data);

    return createSyncedIfDomain(accessor, domain);
}

template<class T, class ObjType>
std::shared_ptr<MemberAccessor<T>>
MemberAccessorFactory::createGetter(
        std::function<T&(ObjType*)> getterRef,
        ObjType* object,
        Domain* domain)
{
    std::shared_ptr<MemberAccessor<T>> accessor =
            std::make_shared<MemberAccessorGetter<T, ObjType>>(
                getterRef, object);

    return createSyncedIfDomain(accessor, domain);
}

template<class T, class ObjType>
std::shared_ptr<MemberAccessor<T>>
MemberAccessorFactory::createGetterSetter(
        std::function<T (ObjType*)> getter,
        std::function<void (ObjType*, T)> setter,
        T defaultValue,
        ObjType* object,
        Domain* domain)
{
    std::shared_ptr<MemberAccessor<T>> accessor =
            std::make_shared<MemberAccessorGetterSetter<T, ObjType>>(
                getter, setter, defaultValue, object);

    return createSyncedIfDomain(accessor, domain);
}

template<class T>
std::shared_ptr<MemberAccessor<T>>
MemberAccessorFactory::createSyncedIfDomain(
        const std::shared_ptr<MemberAccessor<T>>& accessor,
        Domain* domain)
{
    if (domain)
    {
        if (accessor->getType() == MemberAccessorType::OWNER_MEMBER_ACCESSOR)
        {
            return std::make_shared<SyncedOwnerMemberAccessor<T> >(
                        domain, std::static_pointer_cast<OwnerMemberAccessor<T> >(accessor));
        }
        else
        {
            return std::make_shared<SyncedMemberAccessor<T>>(domain, accessor);
        }
    }

    return accessor;
}
