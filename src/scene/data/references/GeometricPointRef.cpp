#include "GeometricPointRef.h"



GeometricPointRef::GeometricPointRef(GeometricData* geometricData, Type type)
    : mGeometricData(geometricData)
    , mType(type)
{

}

GeometricPointRef::~GeometricPointRef()
{

}

GeometricData* GeometricPointRef::getGeometricData()
{
    return mGeometricData;
}


