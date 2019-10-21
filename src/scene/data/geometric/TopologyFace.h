#ifndef TOPOLOGYFACE_H
#define TOPOLOGYFACE_H

#include "TopologyFeature.h"

#include <data_structures/DataStructures.h>


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
};

#endif // TOPOLOGYFACE_H
