#ifndef POLYGONDATA_H
#define POLYGONDATA_H

#include "Polygon.h"

class PolygonData
{
public:
    PolygonData();
    virtual ~PolygonData();

    virtual Polygon::DimensionType getDimensionType() const = 0;

    // Removes a vector at the given index.
    virtual void removeVector(ID index) = 0;

    // Removes the vectors at the given indices.
    virtual void removeVectors(std::vector<ID>& indices) = 0;
};

#endif // POLYGONDATA_H
