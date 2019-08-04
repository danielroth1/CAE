#ifndef MEMBERACCESSORINTERFACE_H
#define MEMBERACCESSORINTERFACE_H

enum class MemberAccessorType
{
    MEMBER_ACCESSOR_INTERFACE, OWNER_MEMBER_ACCESSOR_INTERFACE
};

template<class T>
class MemberAccessorInterface
{
public:
    MemberAccessorInterface();
    virtual ~MemberAccessorInterface();

    virtual bool operator==(MemberAccessorInterface<T>& a);

    virtual T getData() = 0;

    virtual void setData(T data) = 0;

    virtual MemberAccessorType getType() const = 0;
};

#include "MemberAccessorInterface.cpp"

#endif // MEMBERACCESSORINTERFACE_H
