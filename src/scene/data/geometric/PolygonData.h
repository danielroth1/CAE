#ifndef POLYGONDATA_H
#define POLYGONDATA_H

#include "Polygon.h"

class PolygonData
{
public:
    PolygonData();
    virtual ~PolygonData();

    virtual Polygon::DimensionType getDimensionType() const = 0;
};

#endif // POLYGONDATA_H
