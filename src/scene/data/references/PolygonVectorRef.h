#ifndef POLYGONVECTORREF_H
#define POLYGONVECTORREF_H

#include "GeometricPointRef.h"

#include <data_structures/DataStructures.h>

class Polygon;

class PolygonVectorRef : public GeometricPointRef
{
public:
    PolygonVectorRef(Polygon* polygon, Eigen::Vector r);

    Eigen::Vector getR() const;

    void setR(Eigen::Vector r);

    // GeometricPointRef interface
public:
    virtual Type getType() override;

    virtual Vector getPoint() const override;

    // Retruns an GeometricPointRef::ILLEGAL_INDEX
    virtual ID getIndex() const override;

    virtual GeometricPointRef* clone() override;

    virtual void accept(GeometricPointRefVisitor& visitor) override;

private:

    Polygon* mPolygon;

    Eigen::Vector mR;

};

#endif // POLYGONVECTORREF_H
