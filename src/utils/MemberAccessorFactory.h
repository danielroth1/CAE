#ifndef MEMBERACCESSORFACTORY_H
#define MEMBERACCESSORFACTORY_H

#include "MemberAccessor.h"

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

    template<class T>
    static std::shared_ptr<MemberAccessorInterface<T>> createDirect(
            T* data,
            Domain* domain = nullptr);

    template<class T, class ObjType>
    static std::shared_ptr<MemberAccessorInterface<T>> createGetter(
            std::function<T&(ObjType*)> getterRef,
            ObjType* object = nullptr,
            Domain* domain = nullptr);

    template<class T, class ObjType>
    static std::shared_ptr<MemberAccessorInterface<T>> createGetterSetter(
            std::function<T(ObjType*)> getter,
            std::function<void(ObjType*, T)> setter,
            T defaultValue,
            ObjType* object = nullptr,
            Domain* domain = nullptr);

    // Creates a SyncedMemberAccessor if domain != nullptr, else
    // returns \param accessor.
    template<class T>
    static std::shared_ptr<MemberAccessorInterface<T>> createSyncedIfDomain(
            const std::shared_ptr<MemberAccessor<T>>& accessor,
            Domain* domain = nullptr);
};

#include "MemberAccessorFactory.cpp"

#endif // MEMBERACCESSORFACTORY_H
