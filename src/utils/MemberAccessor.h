#ifndef MEMBERACCESSOR_H
#define MEMBERACCESSOR_H

#include "MemberAccessorInterface.h"
#include <functional>
#include <memory>

// A MemberAccessor<T> provides functionality to access the data of variables
// of class T.
//
// There are different types of member accessors. Use getType() to distinguish
// between them.
// Types:
// - MEMBER_ACCESSOR: Accessor that is bound to an object. It can access any
//      kind of variable.
// - OWNER_MEMBER_ACCESSOR:
// One MemberAccessor can access multiple variables
template <class T>
class MemberAccessor : public virtual MemberAccessorInterface<T>
{
public:

    // \param comparator - equality comparator for the accessed data. Is used
    //      in the operation== method.
    MemberAccessor(
            const std::shared_ptr<std::function<bool(T, T)>>& comparator = nullptr);
    virtual ~MemberAccessor();

    // MemberAccessorInterface interface
public:

    virtual bool operator==(MemberAccessorInterface<T>& a);

    virtual T getData() = 0;

    virtual void setData(T data) = 0;

    // Returns the Accessor type. Use this to distinguish between different
    // MemberAccessor implementations.
    virtual MemberAccessorType getType() const;

protected:
    std::shared_ptr<std::function<bool(T, T)>> mEqualityComparator;
};

#include "MemberAccessor.cpp"

#endif // MEMBERACCESSOR_H
