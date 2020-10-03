#include "GeometricPointRefVisitor.h"
#include "PolygonVectorRef.h"

#include <scene/data/geometric/AbstractPolygon.h>

PolygonVectorRef::PolygonVectorRef(
        AbstractPolygon* polygon,
        Vector r,
        BSWSVectors::Type type)
    : GeometricPointRef(polygon, Type::POLYGON_VECTOR)
    , mPolygon(polygon)
    , mR(r)
{

}

void PolygonVectorRef::setR(const Vector& r)
{
    mR = r;
}

Vector PolygonVectorRef::getR() const
{
    return mR;
}

Vector PolygonVectorRef::getPoint() const
{
    switch(mPolygon->getPositionType())
    {
    case BSWSVectors::BODY_SPACE:
        return mPolygon->getTransform() * mR;
    case BSWSVectors::WORLD_SPACE:
        return mR;
    }
    return Vector::Zero();
}

ID PolygonVectorRef::getIndex() const
{
    return ILLEGAL_INDEX;
}

GeometricPointRef* PolygonVectorRef::clone()
{
    return new PolygonVectorRef(mPolygon, mR);
//    return new PolygonVectorRef(*this);
}

void PolygonVectorRef::accept(GeometricPointRefVisitor& visitor)
{
    visitor.visit(*this);
}
