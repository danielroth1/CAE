#ifndef OWNERMEMBERACCESSORINTERFACE_H
#define OWNERMEMBERACCESSORINTERFACE_H

#include "MemberAccessorInterface.h"

#include <functional>
#include <vector>

template<class T>
class OwnerMemberAccessorInterface : public virtual MemberAccessorInterface<T>
{
public:
    using MemberAccessorInterface<T>::getData;
    using MemberAccessorInterface<T>::setData;

    OwnerMemberAccessorInterface();
    virtual ~OwnerMemberAccessorInterface();

    virtual T getData(size_t ownerIndex) = 0;

    virtual void setData(T data, size_t ownerIndex) = 0;

    // Returns the number of owners.
    virtual size_t getNumOwners() const = 0;

    // Returns a vector to all owners of this accessor.
    virtual const std::vector<void*>& getOwners() const = 0;

    // Sets the owners of this accessor.
    virtual void setOwners(const std::vector<void*>& owners) = 0;

    virtual void addOwner(void* owner) = 0;
    virtual void removeOwner(void* owner) = 0;
    virtual void clearOwners() = 0;

    // Sets the data that this accessor accesses.
    virtual void setData(const std::vector<T>& data) = 0;

    virtual T getDefaultValue() = 0;

    virtual void setDefaultValue(T defaultValue) = 0;

    virtual bool isAccessorValuesIdentical() = 0;
};

#include "OwnerMemberAccessorInterface.cpp"

#endif // OWNERMEMBERACCESSORINTERFACE_H
