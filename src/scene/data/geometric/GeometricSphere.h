#ifndef GEOMETRICSPHERE_H
#define GEOMETRICSPHERE_H

#include <memory>

class Polygon2D;


// This is a wrapper class for a Polygon2D whichs
// vertices represent a sphere. This class provides
// a method to change the radius of the sphere. Despite
// that the polygon can be accessed normally.
//
// Note: only use the BODY_SPACE representation for the
// polygon. This class updates the body space vertices
// when the radius changed.
class GeometricSphere
{
public:
    GeometricSphere(double radius, int resolution);

    GeometricSphere(const GeometricSphere& gs);

    std::shared_ptr<Polygon2D> getPolygon();

    // Sets the radius and scales the transformation matrix so that the
    // resulting sphere has the given radius.
    void setRadiusAndScale(double radius);

private:

    std::shared_ptr<Polygon2D> mPolygon;

    double mRadius;
};

#endif // GEOMETRICSPHERE_H
