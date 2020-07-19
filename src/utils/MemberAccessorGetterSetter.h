#ifndef MEMBERACCESSORGETTERSETTER_H
#define MEMBERACCESSORGETTERSETTER_H

#include "MemberAccessor.h"
#include "OwnerMemberAccessor.h"

#include <functional>

// It is recommended to use \class MemberAccessorFactory to create the accessor.
template <class T, class OwnerType>
class MemberAccessorGetterSetter : public OwnerMemberAccessor<T>
{
public:
    MemberAccessorGetterSetter(
            std::function<T(OwnerType*)> getter,
            std::function<void(OwnerType*, T)> setter,
            T defaultValue,
            OwnerType* object = nullptr,
            const std::shared_ptr<std::function<bool(T, T)>>& comparator = nullptr);

    virtual ~MemberAccessorGetterSetter();

    // MemberAccessor
public:
    virtual T getData(size_t ownerIndex);
    virtual void setData(T data, size_t ownerIndex);

    // OwnerMemberAccessor
public:
    void addOwnerWithType(OwnerType* owner);
    OwnerType* getOwnerWithType(size_t ownerIndex);

private:
    std::function<T(OwnerType*)> mGetter;
    std::function<void(OwnerType*, T)> mSetter;
};

#include "MemberAccessorGetterSetter.cpp"

#endif // MEMBERACCESSORGETTERSETTER_H
