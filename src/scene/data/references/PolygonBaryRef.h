#ifndef POLYGONBARYREF_H
#define POLYGONBARYREF_H

#include "GeometricPointRef.h"

#include <array>
#include <data_structures/DataStructures.h>

class Polygon3D;

class PolygonBaryRef : public GeometricPointRef
{
public:
    PolygonBaryRef(Polygon3D* polygon,
                   const std::array<double, 4>& bary,
                   ID elementId);

    // Returns the barycentric coordinates that describe the point w.r.t.
    // the element.
    const std::array<double, 4>& getBary() const
    {
        return mBary;
    }

    ID getElementId() const
    {
        return mElementId;
    }

    // GeometricPointRef interface
public:
    Vector getPoint() const;
    ID getIndex() const;
    GeometricPointRef* clone();
    void accept(GeometricPointRefVisitor& visitor);

private:
    Polygon3D* mPolygon;
    std::array<double, 4> mBary;
    ID mElementId;
};

#endif // POLYGONBARYREF_H
