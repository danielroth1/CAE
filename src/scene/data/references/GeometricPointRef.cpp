#include "GeometricPointRef.h"

GeometricPointRef::GeometricPointRef(GeometricData* geometricData)
    : mGeometricData(geometricData)
{

}

GeometricPointRef::~GeometricPointRef()
{

}

GeometricData* GeometricPointRef::getGeometricData()
{
    return mGeometricData;
}
