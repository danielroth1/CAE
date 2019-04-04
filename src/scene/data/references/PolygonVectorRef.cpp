#include "GeometricPointRefVisitor.h"
#include "PolygonVectorRef.h"

#include <scene/data/geometric/Polygon.h>

PolygonVectorRef::PolygonVectorRef(Polygon* polygon, Vector r)
    : GeometricPointRef(polygon)
    , mPolygon(polygon)
    , mR(r)
{

}

Vector PolygonVectorRef::getR() const
{
    return mR;
}

GeometricPointRef::Type PolygonVectorRef::getType()
{
    return Type::POLYGON_VECTOR;
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

GeometricPointRef* PolygonVectorRef::clone()
{
    return new PolygonVectorRef(mPolygon, mR);
//    return new PolygonVectorRef(*this);
}

void PolygonVectorRef::accept(GeometricPointRefVisitor& visitor)
{
    visitor.visit(*this);
}
