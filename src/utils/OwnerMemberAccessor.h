#ifndef OWNERMEMBERACCESSOR_H
#define OWNERMEMBERACCESSOR_H

#include "MemberAccessor.h"

template <class T>
class OwnerMemberAccessor : public MemberAccessor<T>
{
public:
    // \param T defaultValue - this value is returned if owner is nullptr.
    OwnerMemberAccessor(T defaultValue, void* owner = nullptr);
    virtual ~OwnerMemberAccessor() override;

    // MemberAccessor
public:
    virtual bool hasOwner() const override;

    // Returns a pointer of type void* that points to the owner.
    //
    // For some reasonse these methods need to be declared here.
    // Else the compiler complains about the override. It works
    // for hadOwner() so it probably has to do with the void*.
    virtual void* getOwner() override;

    virtual void setOwner(void* owner) override;

    T getDefaultValue();

    void setDefaultValue(T defaultValue);

protected:
    void* mOwner;

    T mDefaultValue;

};

#include "OwnerMemberAccessor.cpp"

#endif // OWNERMEMBERACCESSOR_H
