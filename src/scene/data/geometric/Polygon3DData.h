#ifndef POLYGON3DDATA_H
#define POLYGON3DDATA_H

#include "data_structures/DataStructures.h"

#include "PolygonData.h"

class Polygon3DTopology;


class Polygon3DData : public PolygonData
{
public:
    Polygon3DData(
            const std::vector<unsigned int>& outerVertexIds,
            const Edges& edges,
            const Edges& outerEdges,
            const Faces& faces,
            const Faces& outerFaces,
            const Cells& cells);

    virtual ~Polygon3DData();

    std::vector<unsigned int>& getOuterVertexIds();
    void setOuterVertexIds(const std::vector<unsigned int>& outerVertexIds);

    Edges& getEdges();
    void setEdges(const Edges& edges);

    Edges& getOuterEdges();
    void setOuterEdges(const Edges& outerEdges);

    Faces& getFaces();
    void setFaces(const Faces& faces);

    Faces& getOuterFaces();
    void setOuterFaces(const Faces& outerFaces);

    Cells& getCells();
    void setCells(const Cells& cells);

    // PolygonData interface
public:
    virtual Polygon::Type getType() const;

private:

    std::unique_ptr<Polygon3DTopology> mTopology;

};

#endif // POLYGON3DDATA_H
