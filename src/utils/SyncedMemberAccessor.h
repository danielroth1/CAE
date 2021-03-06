#ifndef SYNCEDMEMBERACCESSOR_H
#define SYNCEDMEMBERACCESSOR_H


#include "MemberAccessor.h"

#include <memory>

class Domain;


// It is recommended to use \class MemberAccessorFactory to create the accessor.
//
// Wrapper of member accessor that does synchronized setter calls.
// A synchronized call is a call that is executed on a different thread, more specifically
// in that others thread operation queue which is processed at the start of each iteration.
// This access method allows for a thread safe access of data that should only be accessed
// in a specific thread.
// The SyncedMemberAccessor requires the Domain, that is the class whichc holds the thread
// in which the data is accessed.
//
template<class T>
class SyncedMemberAccessor : public virtual MemberAccessorInterface<T>
{
public:
    SyncedMemberAccessor(
            Domain* domain,
            const std::shared_ptr<MemberAccessorInterface<T>>& memberAccessor);

    virtual ~SyncedMemberAccessor() override;

    // MemberAccessor methods
public:

    virtual bool operator==(MemberAccessorInterface<T>& a) override;

    virtual T getData() override;

    virtual void setData(T data) override;

    virtual MemberAccessorType getType() const override;

private:
    Domain* mDomain;

    std::shared_ptr<MemberAccessorInterface<T>> mMemberAccessor;
};

#include "SyncedMemberAccessor.cpp"

#endif // SYNCEDMEMBERACCESSOR_H
