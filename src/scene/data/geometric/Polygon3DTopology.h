#ifndef POLYGON3DTOPOLOGY_H
#define POLYGON3DTOPOLOGY_H

#include "Polygon2DTopology.h"
#include "TopologyCell.h"

#include <data_structures/DataStructures.h>

class Polygon3DTopology : public PolygonTopology
{
public:

    // \param faces - vector of all faces that store indices w.r.t. to the vector
    //      of all vertices.
    // \param outerFaces - vector of Faces. The faces store indices w.r.t. to
    //      the vector of ALL vertices. Internally, the indices are stored w.r.t.
    //      OUTER vertices.
    // \param cells - all cells
    // \param nVertices - number of all vertices (inner and outer).
    Polygon3DTopology(
            const Faces& faces,
            const Faces& outerFaces,
            const Cells& cells,
            ID nVertices);

    Cells& getCellIds();

    std::vector<TopologyCell>& getCells();

    // Outer topology methods
        // Outer topology represents the topology of the outer mesh. All its
        // indices are w.r.t. OUTER vertices.
        // Use getOuterVertexIds()[index] to obtain an index for the vector
        // of ALL vertices.
        Polygon2DTopology& getOuterTopology();
        const Polygon2DTopology& getOuterTopology() const;

        // Maps each index that is w.r.t. OUTER vertices to an index
        // that is w.r.t. ALL vertices.
        std::vector<unsigned int>& getOuterVertexIds();
        void setOuterVertexIds(const std::vector<unsigned int>& outerVertexIds);

        // Maps each index that is w.r.t. OUTER vertices to an index
        // that is w.r.t. ALL vertices.
        std::vector<unsigned int>& getOuterFaceIds();
        void setOuterFaceIds(const std::vector<unsigned int>& outerFaceIds);

        // Returns outer edges. Stored indices are w.r.t. to OUTER vertices.
        // Use getOuterVertexIds()[index] to obtain an index for the vector
        // of ALL vertices.
        std::vector<TopologyEdge>& getOuterEdges();
        const std::vector<TopologyEdge>& getOuterEdges() const;

        // Returns outer faces. Store indices are w.r.t. to OUTER vertices.
        // Use getOuterVertexIds()[index] to obtain an index for the vector
        // of ALL vertices.
        std::vector<TopologyFace>& getOuterFaces();
        const std::vector<TopologyFace>& getOuterFaces() const;

        // Returns the outer faces as index vector: vector<std::array<unsigned int>>
        // Indices point to positions of vector of OUTER positions.
        Faces& getOuterFacesIndices();
        const Faces& getOuterFacesIndices() const;

        // Returns the outer faces as index vector: vector<std::array<unsigned int>>
        // Indices point to positions of vector of ALL positions.
        Faces& getOuterFacesIndices3D();
        const Faces& getOuterFacesIndices3D() const;

        Edges retrieveOuterEdges() const;

        ID to3DVertexIndex(ID index3) const
        {
            return mOuterVertexIds[index3];
        }

        ID to3DFaceIndex(ID index3) const
        {
            return mOuterFaceIds[index3];
        }

private:

    // Adds the cell information to the topology.
    void init();

    // Calculates the IDs of all vertices that are part of the
    // outer hull, which are all vertices that are obtained by
    // mOuterFaces. No index is duplicated.
    std::vector<unsigned int> calculateOuterVertexIDs(const Faces& faces);

    // For each face the stored indices are brought from 3d to 2d
    // representation.
    // \param faces - a vector of faces. The faces store the indices of the
    //      vertices w.r.t. to the 3d vector.
    // \param outerVertexIds - the outerVertexIds that store for each vertex
    //      the corresponding 2d index
    //      outerVertexIds[2d] = 3d
    Faces transformTo2DIndices(
            const Faces& faces,
            const std::vector<unsigned int>& outerVertexIds) const;

    // size = number of outer vertices
    // stores for each outer vertex the corresponding id of all vertices
    std::vector<unsigned int> mOuterVertexIds;

    // size = number of outer face
    // stores for each outer face the corresponding id of all faces
    std::vector<unsigned int> mOuterFaceIds;

    // Outer faces whichs indices point to all positions
    Faces mOuterFacesIndices3D;

    // All cells.
    Cells mCellIds;

    std::vector<TopologyCell> mCells;

    // All indices that are stored here, are w.r.t. to the 2D representation
    // of the outer polygon. To access the vertex position, it is necessary to
    // transform the vertex id to 3d first, so to get the position of a vertex at <id>:
    // mPolygon->getPosition(getOuterVertexIds()[mOuterTopology->getVertex(<id>)->getId()])
    // or alternatively:
    // getPosition(getOuterVertexIds()[<id>])
    // To access outer vertex normals, ths is not necessary:
    // mPolygon->
    Polygon2DTopology mOuterTopology;
};

#endif // POLYGON3DTOPOLOGY_H
