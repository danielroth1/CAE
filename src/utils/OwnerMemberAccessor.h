#ifndef OWNERMEMBERACCESSOR_H
#define OWNERMEMBERACCESSOR_H

#include "MemberAccessor.h"

#include <vector>

// Abstract class for accessors that store one owner.
template <class T>
class OwnerMemberAccessor : public MemberAccessor<T>
{
public:
    // \param T defaultValue - this value is returned if owner is nullptr.
    OwnerMemberAccessor(T defaultValue, void* owner = nullptr);
    virtual ~OwnerMemberAccessor() override;

    // MemberAccessor
public:

    // Returns the data that this accessor accesses. If there is more than
    // one owner (getNumOwners() > 1), the default value (getDefaultValue())
    // is returned.
    virtual T getData() override;

    virtual void setData(T data) override;

    // Returns the Accessor type. Use this to distinguish between different
    // MemberAccessor implementations.
    MemberAccessorType getType() const override;

public:

    virtual T getData(size_t ownerIndex) = 0;

    virtual void setData(T data, size_t ownerIndex) = 0;

    // Returns the number of owners.
    virtual size_t getNumOwners() const;

    // Returns a vector to all owners of this accessor.
    virtual const std::vector<void*>& getOwners() const;

    // Sets the owners of this accessor.
    virtual void setOwners(const std::vector<void*>& owners);

    virtual void addOwner(void* owner);
    virtual void removeOwner(void* owner);
    virtual void clearOwners();

    // Sets the data that this accessor accesses.
    virtual void setData(const std::vector<T>& data);

    virtual T getDefaultValue();

    virtual void setDefaultValue(T defaultValue);

    virtual bool isAccessorValuesIdentical();

protected:
    std::vector<void*> mOwners;

    T mDefaultValue;

};

#include "OwnerMemberAccessor.cpp"

#endif // OWNERMEMBERACCESSOR_H
