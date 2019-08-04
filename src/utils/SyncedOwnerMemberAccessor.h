#ifndef SYNCEDOWNERMEMBERACCESSOR_H
#define SYNCEDOWNERMEMBERACCESSOR_H

#include "OwnerMemberAccessorInterface.h"
#include <memory>

class Domain;

template<class T>
class SyncedOwnerMemberAccessor : public OwnerMemberAccessorInterface<T>
{
public:
    SyncedOwnerMemberAccessor(
            Domain* domain,
            const std::shared_ptr<OwnerMemberAccessorInterface<T>>& accessor);

    // MemberAccessor interface
public:
    virtual T getData();
    virtual void setData(T data);
    virtual MemberAccessorType getType() const;

    // OwnerMemberAccessor interface
public:
    virtual T getData(size_t ownerIndex);
    virtual void setData(T data, size_t ownerIndex);

    virtual size_t getNumOwners() const;
    virtual const std::vector<void*>& getOwners() const;
    virtual void setOwners(const std::vector<void*>& owners);
    virtual void addOwner(void* owner);
    virtual void removeOwner(void* owner);
    virtual void clearOwners();
    virtual void setData(const std::vector<T>& data);
    virtual T getDefaultValue();
    virtual void setDefaultValue(T defaultValue);
    virtual bool isAccessorValuesIdentical();

private:
    Domain* mDomain;
    std::shared_ptr<OwnerMemberAccessorInterface<T>> mAccessor;
};

#include "SyncedOwnerMemberAccessor.cpp"

#endif // SYNCEDOWNERMEMBERACCESSOR_H
