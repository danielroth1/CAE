#ifndef TOPOLOGYFACE_H
#define TOPOLOGYFACE_H

#include "TopologyFeature.h"

#include <data_structures/DataStructures.h>
#include <bitset>

class TopologyFace : public TopologyFeature
{
public:
    TopologyFace(ID id);

    std::array<unsigned int, 3>& getVertexIds()
    {
        return mVertices;
    }
    const std::array<unsigned int, 3>& getVertexIds() const
    {
        return mVertices;
    }

    std::array<unsigned int, 3>& getEdgeIds()
    {
        return mEdges;
    }
    const std::array<unsigned int, 3>& getEdgeIds() const
    {
        return mEdges;
    }

    std::vector<ID>& getCellIds()
    {
        return mCells;
    }
    const std::vector<ID>& getCellIds() const
    {
        return mCells;
    }

    std::vector<ID>& getAdjacentFaces()
    {
        return mAdjacentFaces;
    }
    const std::vector<ID>& getAdjacentFaces() const
    {
        return mAdjacentFaces;
    }

    bool isVertexOwner(int index) const
    {
        return mFeatureOwnership[index];
    }

    bool isEdgeOwner(int index) const
    {
        return mFeatureOwnership[index + 3];
    }

    // Defines this triangle as the owner (or not) of the vertex at the given index.
    // \param index - vertex index between, must be 0, 1, or 2
    void setVertexOwner(int index, bool owner)
    {
        mFeatureOwnership[index] = owner;
    }

    // Defines this triangle as the owner (or not) of the edge at the given index.
    // \param index - edge index between, must be 0, 1, or 2
    void setEdgeOwner(int index, bool owner)
    {
        mFeatureOwnership[index + 3] = owner;
    }

    // TopologyFeature interface
public:
    virtual Type getType() const
    {
        return Type::FACE;
    }

private:
    std::array<unsigned int, 3> mVertices;
    std::array<unsigned int, 3> mEdges;
    std::vector<ID> mCells;

    std::vector<ID> mAdjacentFaces;

    // Defines ownership according to
    // Curtis et al., "Fast Collision Detection for Deformable Models using Representative-Triangles"
    // See "Representative-Triangles" in the class documentation of PolygonTopology.
    //
    // First 6 bytes represent the fact if the face owns one of the adjacent features:
    // v1 v2 v3 e1 e2 e3 0 0 ...
    // v are the vertices and e the edges.
    // The remaining bits are zero.
    std::bitset<6> mFeatureOwnership;
};

#endif // TOPOLOGYFACE_H
