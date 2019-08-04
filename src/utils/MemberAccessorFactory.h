#ifndef MEMBERACCESSORFACTORY_H
#define MEMBERACCESSORFACTORY_H

#include "MemberAccessor.h"

#include <Eigen/Core>
#include <memory>

class Domain;

// A factory to create accessors for
// direct member access (access by memory location),
// reference getter access (access by a getter that returns a reference),
// getter and setter access (access by a getter and a setter).
//
class MemberAccessorFactory
{
public:

    // Creates a MemberAccessorDirect. This accessor is used to directly
    // access members. Use it when only the members memory location is known.
    //
    // \param comparator - equality comparator which is used to check if the
    //      accessed data is equal. If none is provided, the usual operator==
    //      of T is used. Use this comparator to account for numerical errors
    //      avoiding the use of == with float or double.
    template<class T>
    static std::shared_ptr<MemberAccessorInterface<T>> createDirect(
            T* data,
            const std::shared_ptr<std::function<bool(T, T)>>& comparator = nullptr,
            Domain* domain = nullptr);

    // Creates a MemberAccessorGetter. This member accessors allows the access of the
    // data via a getter. Use it if only a getter is known or to restrict the
    // usage of a setter.
    //
    // \param comparator - equality comparator which is used to check if the
    //      accessed data is equal. If none is provided, the usual operator==
    //      of T is used. Use this comparator to account for numerical errors
    //      avoiding the use of == with float or double.
    template<class T, class ObjType>
    static std::shared_ptr<MemberAccessorInterface<T>> createGetter(
            std::function<T&(ObjType*)> getterRef,
            ObjType* object = nullptr,
            const std::shared_ptr<std::function<bool(T, T)>>& comparator = nullptr,
            Domain* domain = nullptr);

    // Creates a MemberAccessorGetterSetter. This member accessor allows the access
    // of the data via a getter and a setter. This accessor should be preferred
    // to the direct member accessor because a direct member access via a
    // memory location is in general not a good design choice.
    //
    // \param comparator - equality comparator which is used to check if the
    //      accessed data is equal. If none is provided, the usual operator==
    //      of T is used. Use this comparator to account for numerical errors
    //      avoiding the use of == with float or double.
    template<class T, class ObjType>
    static std::shared_ptr<MemberAccessorInterface<T>> createGetterSetter(
            std::function<T(ObjType*)> getter,
            std::function<void(ObjType*, T)> setter,
            T defaultValue,
            ObjType* object = nullptr,
            const std::shared_ptr<std::function<bool(T, T)>>& comparator = nullptr,
            Domain* domain = nullptr);

    // Creates a SyncedMemberAccessor if domain != nullptr, else
    // returns \param accessor.
    template<class T>
    static std::shared_ptr<MemberAccessorInterface<T>> createSyncedIfDomain(
            const std::shared_ptr<MemberAccessor<T>>& accessor,
            Domain* domain = nullptr);

    static std::shared_ptr<std::function<bool (bool, bool)> >
    createBoolComparator();

    static std::shared_ptr<std::function<bool (int, int)> >
    createIntComparator();

    static std::shared_ptr<std::function<bool(double, double)>>
    createDoubleComparator(double precision = 1e-12);

    static std::shared_ptr<std::function<bool (Eigen::Vector3d, Eigen::Vector3d)> >
    createVectorDoubleComparator(double precision = 1e-12);


};

#include "MemberAccessorFactory.cpp"

#endif // MEMBERACCESSORFACTORY_H
