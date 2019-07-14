#ifndef TOPOLOGYVERTEX_H
#define TOPOLOGYVERTEX_H

#include "TopologyFeature.h"

#include <data_structures/DataStructures.h>

class TopologyVertex : public TopologyFeature
{
public:
    TopologyVertex(ID id);

    std::vector<ID>& getEdgeIds();
    const std::vector<ID>& getEdgeIds() const;
    std::vector<ID>& getFaceIds();
    const std::vector<ID>& getFaceIds() const;
    std::vector<ID>& getCellIds();
    const std::vector<ID>& getCellIds() const;

    // TopologyFeature interface
public:
    virtual Type getType() const;

private:
    std::vector<ID> mEdges;
    std::vector<ID> mFaces;

    // For Topology3D
    std::vector<ID> mCells;
};

#endif // TOPOLOGYVERTEX_H
