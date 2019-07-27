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
            OwnerType* object = nullptr);

    virtual ~MemberAccessorGetterSetter();

    // MemberAccessor
public:
    virtual T getData();
    virtual void setData(T data);

    // OwnerMemberAccessor
public:
    void setOwnerWithType(OwnerType* owner);
    OwnerType* getOwnerWithType();

private:
    std::function<T(OwnerType*)> mGetter;
    std::function<void(OwnerType*, T)> mSetter;
};

#include "MemberAccessorGetterSetter.cpp"

#endif // MEMBERACCESSORGETTERSETTER_H
