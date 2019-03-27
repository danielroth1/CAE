#include "GeometricPointRefVector.h"

GeometricPointRefVector::GeometricPointRefVector(GeometricData* object)
    : PointRefCollectionVector<GeometricData *, Eigen::Vector &> (object)
{

}

GeometricPointRefVector::~GeometricPointRefVector()
{

}

Vector& GeometricPointRefVector::getPoint(ID index)
{
    return mObject->getPosition(index);
}
