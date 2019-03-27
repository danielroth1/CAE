#ifndef POINTREF_H
#define POINTREF_H

#include <data_structures/DataStructures.h>

template <class T, class P>
class PointRef
{

public:
    T getObject();

    ID getPointIndex();

    virtual P getPoint() = 0;

protected:
    // Constructor is protected because Object type is not
    // known. getPoint() must be implemented.
    PointRef(T object, ID pointIndex);

    T mObject;

    ID mIndex;
};

#include "PointRef.cpp"

#endif // POINTREF_H
