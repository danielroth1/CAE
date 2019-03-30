#pragma once

#ifndef MEMBERACCESSORFACTORY_CPP
#define MEMBERACCESSORFACTORY_CPP

#include "MemberAccessorDirect.h"
#include "MemberAccessorFactory.h"
#include "MemberAccessorGetter.h"
#include "MemberAccessorGetterSetter.h"
#include "SyncedMemberAccessor.h"


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
        ObjType* object,
        Domain* domain)
{
    std::shared_ptr<MemberAccessor<T>> accessor =
            std::make_shared<MemberAccessorGetterSetter<T, ObjType>>(
                getter, setter, object);

    return createSyncedIfDomain(accessor, domain);
}

template<class T>
std::shared_ptr<MemberAccessor<T>>
MemberAccessorFactory::createSyncedIfDomain(
        const std::shared_ptr<MemberAccessor<T>>& accessor,
        Domain* domain)
{
    if (domain)
        return std::make_shared<SyncedMemberAccessor<T>>(domain, accessor);

    return accessor;
}
