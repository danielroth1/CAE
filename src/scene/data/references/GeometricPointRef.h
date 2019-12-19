#ifndef GEOMETRICPOINTREF_H
#define GEOMETRICPOINTREF_H

#include <scene/data/GeometricData.h>

//#include <data_structures/references/PointRef.h>

class GeometricPointRefVisitor;

class GeometricPointRef// : public PointRef<GeometricData*, Eigen::Vector&>
{
public:
    enum class Type
    {
        GEOMETRIC_VERTEX, POLYGON_VECTOR
    };

    GeometricPointRef(GeometricData* geometricData, Type type);

    virtual ~GeometricPointRef();

    GeometricData* getGeometricData();

    Type getType() const
    {
        return mType;
    }

    virtual Eigen::Vector getPoint() const = 0;

    // Returns the index of the vertex this reference points to.
    // If this is not a reference that points to vertices, e.g.
    // PolygonVectorRef, ILLEGAL_INDEX is returned instead.
    virtual ID getIndex() const = 0;

    virtual GeometricPointRef* clone() = 0;

    virtual void accept(GeometricPointRefVisitor& visitor) = 0;

protected:
    GeometricData* mGeometricData;

    Type mType;

};

#endif // GEOMETRICPOINTREF_H
