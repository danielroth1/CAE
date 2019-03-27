#include "GeometricPoint.h"

#include <scene/data/GeometricDataVisitor.h>

GeometricPoint::GeometricPoint(Vector position)
    : mPosition(position)
{

}

void GeometricPoint::updateBoundingBox()
{


}

void GeometricPoint::accept(GeometricDataVisitor& visitor)
{
    visitor.visit(*this);
}

Vector& GeometricPoint::getPosition(size_t /*index*/)
{
    return mPosition;
}

Vector&GeometricPoint::getPosition()
{
    return mPosition;
}

size_t GeometricPoint::getSize()
{
    return 1;
}

void GeometricPoint::translate(const Eigen::Vector& position)
{
    mPosition += position;
}
