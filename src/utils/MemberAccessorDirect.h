#ifndef MEMBERACCESSORDIRECT_H
#define MEMBERACCESSORDIRECT_H

#include "MemberAccessor.h"


// It is recommended to use \class MemberAccessorFactory to create the accessor.
template <class T>
class MemberAccessorDirect : public MemberAccessor<T>
{
public:
    MemberAccessorDirect(
            T* data,
            const std::shared_ptr<std::function<bool(T, T)>>& comparator = nullptr);

    virtual ~MemberAccessorDirect() override;

    virtual T getData() override;

    virtual void setData(T data) override;

private:
    T* mDataPtr;

};

#include "MemberAccessor.cpp"

#endif // MEMBERACCESSORDIRECT_H
