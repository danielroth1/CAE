#ifndef TOPOLOGYFACE_H
#define TOPOLOGYFACE_H

#include "TopologyFeature.h"

#include <data_structures/DataStructures.h>


class TopologyFace : public TopologyFeature
{
public:
    TopologyFace(ID id);

    std::array<unsigned int, 3>& getVertexIds();
    const std::array<unsigned int, 3>& getVertexIds() const;
    std::array<unsigned int, 3>& getEdgeIds();
    const std::array<unsigned int, 3>& getEdgeIds() const;
    std::vector<ID>& getAdjacentFaces();
    const std::vector<ID>& getAdjacentFaces() const;

    // TopologyFeature interface
public:
    virtual Type getType() const;

private:
    std::array<unsigned int, 3> mVertices;
    std::array<unsigned int, 3> mEdges;

    std::vector<ID> mAdjacentFaces;
};

#endif // TOPOLOGYFACE_H