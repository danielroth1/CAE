#include "GeometricDataFactory.h"
#include "GeometricSphere.h"
#include "Polygon2D.h"

GeometricSphere::GeometricSphere(double radius, int resolution)
    : mRadius(radius)
{
    mPolygon = std::make_shared<Polygon2D>(GeometricDataFactory::create2DSphere(radius, resolution));
    mPolygon->changeRepresentationToBS(Vector::Zero());
}

GeometricSphere::GeometricSphere(const GeometricSphere& gs)
{
    mPolygon = std::make_shared<Polygon2D>(*gs.mPolygon.get());
    setRadiusAndProject(gs.mRadius);
}

std::shared_ptr<Polygon2D> GeometricSphere::getPolygon()
{
    return mPolygon;
}

void GeometricSphere::setRadiusAndProject(double radius)
{
    mRadius = radius;
//    projectOnSphere(radius);
}

void GeometricSphere::projectOnSphere(double radius)
{
    for (Eigen::Vector& v : mPolygon->getPositionsBS())
    {
        v.normalize();
        v *= radius;
    }
}

