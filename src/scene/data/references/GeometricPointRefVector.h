#ifndef GEOMETRICPOINTREFVECTOR_H
#define GEOMETRICPOINTREFVECTOR_H

#include <scene/data/GeometricData.h>

#include <data_structures/references/PointRefCollectionVector.h>



class GeometricPointRefVector : public PointRefCollectionVector<GeometricData*, Eigen::Vector&>
{
public:
    GeometricPointRefVector(GeometricData* object);

    virtual ~GeometricPointRefVector();

    // PointRefCollectionVector interface
public:
    virtual Eigen::Vector& getPoint(ID index);
};

#endif // GEOMETRICPOINTREFVECTOR_H
