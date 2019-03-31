#ifndef MEMBERACCESSORGETTERSETTER_H
#define MEMBERACCESSORGETTERSETTER_H

#include "MemberAccessor.h"

#include <functional>


// It is recommended to use \class MemberAccessorFactory to create the accessor.
template <class T, class ObjType>
class MemberAccessorGetterSetter : public MemberAccessor<T>
{
public:
    MemberAccessorGetterSetter(
            std::function<T(ObjType*)> getter,
            std::function<void(ObjType*, T)> setter,
            ObjType* object = nullptr);

    virtual ~MemberAccessorGetterSetter();

    void setObject(ObjType* object);

    ObjType* getObject();

    virtual T getData() override;

    virtual void setData(T data) override;

private:
    std::function<T(ObjType*)> mGetter;
    std::function<void(ObjType*, T)> mSetter;

    ObjType* mObject;
};

#include "MemberAccessorGetterSetter.cpp"

#endif // MEMBERACCESSORGETTERSETTER_H
