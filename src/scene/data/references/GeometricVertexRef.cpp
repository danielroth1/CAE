#include "GeometricPointRefVisitor.h"
#include "GeometricVertexRef.h"

#include <scene/data/geometric/Polygon.h>

GeometricVertexRef::GeometricVertexRef(GeometricData* geometricData, ID index)
    : GeometricPointRef(geometricData)
    , mIndex(index)
{

}

GeometricPointRef::Type GeometricVertexRef::getType()
{
    return Type::GEOMETRIC_VERTEX;
}

Vector GeometricVertexRef::getPoint() const
{
    return mGeometricData->getPosition(mIndex);
}

ID GeometricVertexRef::getIndex() const
{
    return mIndex;
}

GeometricPointRef* GeometricVertexRef::clone()
{
    return new GeometricVertexRef(mGeometricData, mIndex);
}

void GeometricVertexRef::accept(GeometricPointRefVisitor& visitor)
{
    visitor.visit(*this);
}
