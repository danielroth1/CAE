#ifndef POLYGON3DDATA_H
#define POLYGON3DDATA_H

#include "data_structures/DataStructures.h"

#include "PolygonData.h"

class Polygon3DTopology;


class Polygon3DData : public PolygonData
{
public:
    Polygon3DData(
            const Faces& faces,
            const Faces& outerFaces,
            const Cells& cells,
            ID nVertices);

    Polygon3DData(const Polygon3DTopology& topology);

    virtual ~Polygon3DData();

    Polygon3DTopology& getTopology();

    // PolygonData interface
public:
    virtual Polygon::Type getType() const;

private:

    std::unique_ptr<Polygon3DTopology> mTopology;

};

#endif // POLYGON3DDATA_H
