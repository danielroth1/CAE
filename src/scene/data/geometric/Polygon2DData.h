#ifndef POLYGON2DDATA_H
#define POLYGON2DDATA_H

#include "BSWSVectors.h"
#include "PolygonData.h"
#include <memory>

class Polygon2DTopology;

class Polygon2DData : public PolygonData
{
public:

    Polygon2DData(const Polygon2DTopology& topology);

    virtual ~Polygon2DData() override;

    virtual BSWSVectors::Type getPositionType() = 0;

    Polygon2DTopology& getTopology();

    // PolygonData interface
public:
    virtual Polygon::DimensionType getDimensionType() const override;

    // Removes a vector at the given index.
    virtual void removeVector(ID index) override;

    // Removes the vectors at the given indices.
    virtual void removeVectors(std::vector<ID>& indices) override;

private:
    std::unique_ptr<Polygon2DTopology> mTopology;
};

#endif // POLYGON2DDATA_H
