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
//
// Representative-Triangles:
// Sets for each face the feature owner ships, see
// Curtis et al. "Fast Collision Detection for Deformable Models using Representative-Triangles"
//
// The idea is that each vertex and each edge is owned by only one face.
// Features that are shared by multiple faces are only assigned to one face.
// This can be used to speed up things like collision detection because it
// allows to avoid collision duplications. When iterating over triangle, only
// consider the features that the triangle owns.
//
class PolygonTopology
{
public:
    // Default constructor
    PolygonTopology();

    PolygonTopology(const Faces& faces, ID nVertices);
    virtual ~PolygonTopology();

    // Removes all given vertices. If one vertex is part of an other topological
    // element, that element is removed as well.
    virtual void removeVertices(std::vector<ID>& vertexIds);

    std::vector<ID> retrieveNotReferencedByEdges() const;
    std::vector<ID> retrieveNotReferencedByFaces() const;

    TopologyVertex& getVertex(ID id)
    {
        return mVertices[id];
    }
    TopologyEdge& getEdge(ID id)
    {
        return mEdges[id];
    }
    TopologyFace& getFace(ID id)
    {
        return mFaces[id];
    }

    std::vector<TopologyVertex>& getVertices()
    {
        return mVertices;
    }
    const std::vector<TopologyVertex>& getVertices() const
    {
        return mVertices;
    }
    std::vector<TopologyEdge>& getEdges()
    {
        return mEdges;
    }
    const std::vector<TopologyEdge>& getEdges() const
    {
        return mEdges;
    }
    std::vector<TopologyFace>& getFaces()
    {
        return mFaces;
    }
    const std::vector<TopologyFace>& getFaces() const
    {
        return mFaces;
    }
    Faces& getFacesIndices()
    {
        return mFacesIndices;
    }
    const Faces& getFacesIndices() const
    {
        return mFacesIndices;
    }

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
    // vertices specified in removedIds would be removed. Replaces removed
    // vertex indices by 0. if removedIds is empty, an empty vector is
    // retruned. Sorts removedIds by ascending indices.
    //
    // Example:
    // original: 0 1 2 3 4 5 6 7 8 9
    // removed:  3 7
    // new:      0 1 2 3 4 5 6 7
    // oldToNew: 0 1 2 0 3 4 5 0 6 7
    template<class T>
    std::vector<T> createOldToNewMapping(const std::vector<T>& removedIds,
                                         size_t nVerticesBefore) const
    {
        std::vector<T> oldToNew;
        std::vector<T> removeIdsCopy = removedIds;
        if (removeIdsCopy.empty())
            return oldToNew;

        std::sort(removeIdsCopy.begin(), removeIdsCopy.end());

        oldToNew.resize(nVerticesBefore);

        T removedIt = 0;
        for (T i = 0; i < nVerticesBefore; ++i)
        {
            if (removedIt < removeIdsCopy.size() && removeIdsCopy[removedIt] == i)
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

    // Implements a greedy algorithm. Iterate each face and add assign all
    // features as owner that are not already assigned.
    void setFaceOwnerships();

    TopologyEdge createEdge(ID f1, ID f2);

    std::pair<ID, ID> makeSmallerPair(ID i1, ID i2) const;
};

#endif // POLYGONTOPOLOGY_H
