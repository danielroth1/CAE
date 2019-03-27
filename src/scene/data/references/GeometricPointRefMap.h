#ifndef GEOMETRICPOINTREFMAP_H
#define GEOMETRICPOINTREFMAP_H

#include <scene/data/GeometricData.h>

#include <data_structures/references/PointRefCollectionMap.h>



class GeometricPointRefMap : public PointRefCollectionMap<GeometricData*, Eigen::Vector&>
{
public:
    GeometricPointRefMap();

    virtual ~GeometricPointRefMap();

    // PointRefCollectionMap interface
public:
    virtual Eigen::Vector& getPoint(GeometricData* object, ID index);
};

#endif // GEOMETRICPOINTREFMAP_H
