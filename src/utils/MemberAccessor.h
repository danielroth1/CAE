#ifndef MEMBERACCESSOR_H
#define MEMBERACCESSOR_H

#include "MemberAccessorInterface.h"
#include <functional>

// A MemberAccessor<T> provides functionality to access the data of variables
// of class T.
//
// There are different types of member accessors. Use getType() to distinguish
// between them.
// Types:
// - MEMBER_ACCESSOR: Accessor that is bound to an object. It can access any
//      kind of variable.
// - OWNER_MEMBER_ACCESSOR: Accesses
// One MemberAccessor can access multiple variables
template <class T>
class MemberAccessor : public virtual MemberAccessorInterface<T>
{
public:

    MemberAccessor();
    virtual ~MemberAccessor();

    // MemberAccessorInterface interface
public:
    virtual bool operator==(MemberAccessor<T>& a);

    virtual T getData() = 0;

    virtual void setData(T data) = 0;

    // Returns the Accessor type. Use this to distinguish between different
    // MemberAccessor implementations.
    virtual MemberAccessorType getType() const;
};

#include "MemberAccessor.cpp"

#endif // MEMBERACCESSOR_H
