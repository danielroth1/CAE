#ifndef MEMBERACCESSOR_H
#define MEMBERACCESSOR_H

template <class T>
class MemberAccessor
{
public:
    MemberAccessor();
    virtual ~MemberAccessor();

    virtual T getData() = 0;

    virtual void setData(T data) = 0;
};

#include "MemberAccessor.cpp"

#endif // MEMBERACCESSOR_H
