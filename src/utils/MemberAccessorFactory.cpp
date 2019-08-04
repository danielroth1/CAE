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
std::shared_ptr<MemberAccessorInterface<T>>
MemberAccessorFactory::createDirect(
        T* data,
        const std::shared_ptr<std::function<bool(T, T)>>& comparator,
        Domain* domain)
{
    std::shared_ptr<MemberAccessor<T>> accessor =
            std::make_shared<MemberAccessorDirect<T>>(data, comparator);

    return createSyncedIfDomain(accessor, domain);
}

template<class T, class ObjType>
std::shared_ptr<MemberAccessorInterface<T>>
MemberAccessorFactory::createGetter(
        std::function<T&(ObjType*)> getterRef,
        ObjType* object,
        const std::shared_ptr<std::function<bool(T, T)>>& comparator,
        Domain* domain)
{
    std::shared_ptr<MemberAccessor<T>> accessor =
            std::make_shared<MemberAccessorGetter<T, ObjType>>(
                getterRef, object, comparator);

    return createSyncedIfDomain(accessor, domain);
}

template<class T, class ObjType>
std::shared_ptr<MemberAccessorInterface<T>>
MemberAccessorFactory::createGetterSetter(
        std::function<T (ObjType*)> getter,
        std::function<void (ObjType*, T)> setter,
        T defaultValue,
        ObjType* object,
        const std::shared_ptr<std::function<bool(T, T)>>& comparator,
        Domain* domain)
{
    std::shared_ptr<MemberAccessor<T>> accessor =
            std::make_shared<MemberAccessorGetterSetter<T, ObjType>>(
                getter, setter, defaultValue, object, comparator);

    return createSyncedIfDomain(accessor, domain);
}

template<class T>
std::shared_ptr<MemberAccessorInterface<T>>
MemberAccessorFactory::createSyncedIfDomain(
        const std::shared_ptr<MemberAccessor<T>>& accessor,
        Domain* domain)
{
    if (domain)
    {
        if (accessor->getType() == MemberAccessorType::OWNER_MEMBER_ACCESSOR_INTERFACE)
        {
            return std::make_shared<SyncedOwnerMemberAccessor<T> >(
                        domain, std::dynamic_pointer_cast<OwnerMemberAccessorInterface<T> >(accessor));
        }
        else
        {
            return std::make_shared<SyncedMemberAccessor<T>>(domain, accessor);
        }
    }

    return accessor;
}

std::shared_ptr<std::function<bool (bool, bool)> >
MemberAccessorFactory::createBoolComparator()
{
    return std::make_shared<std::function<bool (bool, bool)> >(
                [](bool a, bool b)
    {
        return a == b;
    });
}

std::shared_ptr<std::function<bool (int, int)> >
MemberAccessorFactory::createIntComparator()
{
    return std::make_shared<std::function<bool (int, int)> >(
                [](int a, int b)
    {
        return a == b;
    });
}

std::shared_ptr<std::function<bool (double, double)> >
MemberAccessorFactory::createDoubleComparator(double precision)
{
    return std::make_shared<std::function<bool (double, double)> >(
                [precision](double a, double b)
    {
        return std::abs(a - b) < precision;
    });
}

std::shared_ptr<std::function<bool (Eigen::Vector3d, Eigen::Vector3d)> >
MemberAccessorFactory::createVectorDoubleComparator(double precision)
{
    return std::make_shared<std::function<bool(Eigen::Vector3d, Eigen::Vector3d)> >(
                [precision](Eigen::Vector3d a, Eigen::Vector3d b)
    {
        return a.isApprox(b, precision);
    });
}
