#ifndef POLYGON3DTOPOLOGY_H
#define POLYGON3DTOPOLOGY_H

#include <data_structures/DataStructures.h>

class Polygon3DTopology
{
public:
    Polygon3DTopology(
            const std::vector<unsigned int>& outerVertexIds,
            const Edges& edges,
            const Edges& outerEdges,
            const Faces& faces,
            const Faces& outerFaces,
            const Cells& cells);

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

private:
    std::vector<unsigned int> mOuterVertexIds;

    // Edges of all triangles, inner and outer.
    Edges mEdges;

    // Edges of all oter faces.
    Edges mOuterEdges;

    // All faces, in- and outside.
    Faces mFaces;

    // All outside faces.
    Faces mOuterFaces;

    // All cells.
    Cells mCells;
};

#endif // POLYGON3DTOPOLOGY_H
