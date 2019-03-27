#include "GeometricPointRefMap.h"

GeometricPointRefMap::GeometricPointRefMap()
{

}

GeometricPointRefMap::~GeometricPointRefMap()
{

}

Vector& GeometricPointRefMap::getPoint(GeometricData* object, ID index)
{
    return object->getPosition(mMap[object][index]);
}
