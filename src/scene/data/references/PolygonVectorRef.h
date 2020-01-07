#ifndef POLYGONVECTORREF_H
#define POLYGONVECTORREF_H

#include "GeometricPointRef.h"

#include <data_structures/DataStructures.h>

#include <scene/data/geometric/BSWSVectors.h>

class Polygon;

// We want a type for Simulation objects and GeometricPointRef?
// Maybe just reuse the mType of GoemetricPointRef
class PolygonVectorRef : public GeometricPointRef
{
public:

    // \param type - the transformation type of the given point r. If
    //      WORLD_SPACE, the point r is returned with getR(). If BODY_SPACE
    //      it is first rotated.
    //      Note: if polygon is stored in BODY_SPACE, type can be both
    //          WORLD_SPACE and BODY_SPACE.
    //          if polygon is stored in WORLD_SPACE, type can only be
    //          WORLD_SPACE.
    PolygonVectorRef(Polygon* polygon,
                     Eigen::Vector r,
                     BSWSVectors::Type type = BSWSVectors::Type::BODY_SPACE);

    void setR(const Eigen::Vector& r);
    Eigen::Vector getR() const;

    // GeometricPointRef interface
public:

    // \param type
    //      if type == WORLD_SPACE: returns r
    //      if type == BODY_SPACE:  returns polygon->R * r
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
