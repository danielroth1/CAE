#ifndef MEMBERACCESSORGETTER_H
#define MEMBERACCESSORGETTER_H

#include "MemberAccessor.h"

#include <functional>


// It is recommended to use \class MemberAccessorFactory to create the accessor.
template <class T, class ObjType>
class MemberAccessorGetter : public MemberAccessor<T>
{
public:
    // \param getterRef - a function to the getter. The getter must return
    // a reference. The data is set with that reference.
    MemberAccessorGetter(
            std::function<T&(ObjType*)> getterRef,
            ObjType* object = nullptr);

    virtual ~MemberAccessorGetter() override;

    void setObject(ObjType* object);

    ObjType* getObject();

    virtual T getData() override;

    virtual void setData(T data) override;

private:
    ObjType* mObject;

    std::function<T&(ObjType*)> mGetterRef;

};

#include "MemberAccessorGetter.cpp"

#endif // MEMBERACCESSORGETTER_H
