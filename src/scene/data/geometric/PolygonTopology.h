#ifndef POLYGONTOPOLOGY_H
#define POLYGONTOPOLOGY_H


#include "TopologyEdge.h"
#include "TopologyFace.h"
#include "TopologyVertex.h"

#include <data_structures/DataStructures.h>


// Given are the 3 vertex ids per face saved in Faces.
// From that are calculated, the:
//  * 3 edges per face
//  * 2 vertices per edge
//  * 2 faces per edge
//  * n edges per vertex
//  * m faces per vertex
class PolygonTopology
{
public:
    PolygonTopology(const Faces& faces, ID nVertices);

    TopologyVertex& getVertex(ID id);
    TopologyEdge& getEdge(ID id);
    TopologyFace& getFace(ID id);

    std::vector<TopologyVertex>& getVertices();
    const std::vector<TopologyVertex>& getVertices() const;
    std::vector<TopologyEdge>& getEdges();
    const std::vector<TopologyEdge>& getEdges() const;
    std::vector<TopologyFace>& getFaces();
    const std::vector<TopologyFace>& getFaces() const;
    Faces& getFacesIndices();
    const Faces& getFacesIndices() const;

    Edges retrieveEdges() const;
    Faces retrieveFaces() const;

    std::string toString() const;

protected:

    // An old inefficient way of calculating edges from faces. Use buildTopology
    // instead.
    // Calcualtes the edges for the given faces.
    // Avoids duplicated edges. For each edges with
    // its vertex indices (i0, i1) it is always
    // i0 < i1. This condition is part of the algorithm
    // and is true for the resulting edges as well.
    std::vector<TopologyEdge> calculateEdges(const Faces& faces) const;

    std::vector<TopologyVertex> mVertices;
    std::vector<TopologyEdge> mEdges;
    std::vector<TopologyFace> mFaces;

    Faces mFacesIndices;

private:

    void buildTopology(
                const Faces& faces,
                ID nVertices,
                std::vector<TopologyVertex>& verticesOut,
                std::vector<TopologyEdge>& edgesOut,
                std::vector<TopologyFace>& facesOut) const;

    TopologyEdge createEdge(ID f1, ID f2);

    std::pair<ID, ID> makeSmallerPair(ID i1, ID i2) const;
};

#endif // POLYGONTOPOLOGY_H
