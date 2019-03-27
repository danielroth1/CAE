#ifndef POLYGON2DDATA_H
#define POLYGON2DDATA_H

#include "BSWSVectors.h"
#include "PolygonData.h"
#include <memory>

class Polygon2DTopology;

class Polygon2DData : public PolygonData
{
public:
    Polygon2DData(const Faces& faces,
                  const Edges& edges);

    virtual ~Polygon2DData() override;

    virtual BSWSVectors::Type getPositionType() = 0;

    Faces& getFaces();
    Edges& getEdges();

    // PolygonData interface
public:
    virtual Polygon::Type getType() const override;

private:
    std::unique_ptr<Polygon2DTopology> mTopology;
};

#endif // POLYGON2DDATA_H
