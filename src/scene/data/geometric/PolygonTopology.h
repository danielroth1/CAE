#ifndef POLYGONTOPOLOGY_H
#define POLYGONTOPOLOGY_H


#include "TopologyEdge.h"
#include "TopologyFace.h"
#include "TopologyVertex.h"

#include <data_structures/DataStructures.h>
#include <vector>

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
    virtual ~PolygonTopology();

    // Removes all given vertices. If one vertex is part of an other topological
    // element, that element is removed as well.
    virtual void removeVertices(std::vector<ID>& vertexIds);

    std::vector<ID> retrieveNotReferencedByEdges() const;
    std::vector<ID> retrieveNotReferencedByFaces() const;

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

    // Intializes this topology from the given faces. Calling this method
    // is like recaling the constructor.
    void init(const Faces& faces, ID nVertices);

    // An old inefficient way of calculating edges from faces. Use buildTopology
    // instead.
    // Calcualtes the edges for the given faces.
    // Avoids duplicated edges. For each edges with
    // its vertex indices (i0, i1) it is always
    // i0 < i1. This condition is part of the algorithm
    // and is true for the resulting edges as well.
    std::vector<TopologyEdge> calculateEdges(const Faces& faces) const;

    // Creates a mapping that maps each vertex ids to its new id after all
    // vertices specified in removedVertexIds would be removed. Replaces removed
    // vertex indices by 0. if removedVertexIds is empty, an empty vector is
    // retruned. Sorts removedVertexIds by ascending indices.
    //
    // Example:
    // original: 0 1 2 3 4 5 6 7 8 9
    // removed:  3 7
    // new:      0 1 2 3 4 5 6 7
    // oldToNew: 0 1 2 0 3 4 5 0 6 7
    template<class T>
    std::vector<T> createOldToNewMapping(std::vector<T>& removedVertexIds)
    {
        std::vector<T> oldToNew;
        if (removedVertexIds.empty())
            return oldToNew;

        std::sort(removedVertexIds.begin(), removedVertexIds.end());

        size_t nVerticesBefore = mVertices.size();

        oldToNew.resize(nVerticesBefore);

        T removedIt = 0;
        for (T i = 0; i < nVerticesBefore; ++i)
        {
            if (removedIt < removedVertexIds.size() && removedVertexIds[removedIt] == i)
            {
                ++removedIt;
                oldToNew[i] = 0;
            }
            else
            {
                oldToNew[i] = i - removedIt;
            }
        }

        return oldToNew;
    }

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
