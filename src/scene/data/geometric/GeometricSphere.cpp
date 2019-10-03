#include "GeometricDataFactory.h"
#include "GeometricSphere.h"
#include "Polygon2D.h"

GeometricSphere::GeometricSphere(double radius, int resolution)
{
    mRadius = -1.0;

    mPolygon = std::make_shared<Polygon2D>(
                GeometricDataFactory::create2DSphere(1.0, resolution));
    mPolygon->changeRepresentationToBS(Vector::Zero());

    setRadiusAndScale(radius);
}

GeometricSphere::GeometricSphere(const GeometricSphere& gs)
{
    mRadius = -1.0;

    mPolygon = std::make_shared<Polygon2D>(*gs.mPolygon.get());

    setRadiusAndScale(gs.mRadius);
}

std::shared_ptr<Polygon2D> GeometricSphere::getPolygon()
{
    return mPolygon;
}

void GeometricSphere::setRadiusAndScale(double radius)
{
    double scalingValue;
    if (mRadius > 0)
        scalingValue = radius / mRadius;
    else
        scalingValue = radius;

    mRadius = radius;
    mPolygon->transform(Eigen::Affine3d(Eigen::Scaling(scalingValue)));
}
