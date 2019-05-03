#ifndef POLYGON3DTOPOLOGY_H
#define POLYGON3DTOPOLOGY_H

#include "Polygon2DTopology.h"

#include <data_structures/DataStructures.h>

class Polygon3DTopology : public PolygonTopology
{
public:
    Polygon3DTopology(
            const Faces& faces,
            const Faces& outerFaces,
            const Cells& cells,
            ID nVertices);

    Cells& getCells();

    // Outer topology methods
        std::vector<unsigned int>& getOuterVertexIds();
        void setOuterVertexIds(const std::vector<unsigned int>& outerVertexIds);

        std::vector<TopologyEdge>& getOuterEdges();
        const std::vector<TopologyEdge>& getOuterEdges() const;
        std::vector<TopologyFace>& getOuterFaces();
        const std::vector<TopologyFace>& getOuterFaces() const;


        Edges retrieveOuterEdges() const;
        Faces retrieveOuterFaces() const;

private:

    // Calculates the IDs of all vertices that are part of the
    // outer hull, which are all vertices that are obtained by
    // mOuterFaces. No index is duplicated.
    std::vector<unsigned int> calculateOuterVertexIDs(const Faces& faces);


    std::vector<unsigned int> mOuterVertexIds;

    // All faces, in- and outside.
    Faces mFaces;

    // All cells.
    Cells mCells;

    Polygon2DTopology mOuterTopology;
};

#endif // POLYGON3DTOPOLOGY_H
