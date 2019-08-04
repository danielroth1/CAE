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
    virtual bool operator==(MemberAccessorInterface<T>& a) override;
    virtual T getData() override;
    virtual void setData(T data) override;
    virtual MemberAccessorType getType() const override;

    // OwnerMemberAccessor interface
public:

    virtual T getData(size_t ownerIndex) override;
    virtual void setData(T data, size_t ownerIndex) override;
    virtual size_t getNumOwners() const override;
    virtual const std::vector<void*>& getOwners() const override;
    virtual void setOwners(const std::vector<void*>& owners) override;
    virtual void addOwner(void* owner) override;
    virtual void removeOwner(void* owner) override;
    virtual void clearOwners() override;
    virtual void setData(const std::vector<T>& data) override;
    virtual T getDefaultValue() override;
    virtual void setDefaultValue(T defaultValue) override;
    virtual bool isAccessorValuesIdentical() override;

private:
    Domain* mDomain;
    std::shared_ptr<OwnerMemberAccessorInterface<T>> mAccessor;
};

#include "SyncedOwnerMemberAccessor.cpp"

#endif // SYNCEDOWNERMEMBERACCESSOR_H
