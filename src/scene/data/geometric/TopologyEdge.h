#ifndef TOPOLOGYEDGE_H
#define TOPOLOGYEDGE_H

#include "TopologyFeature.h"

#include <data_structures/DataStructures.h>

class TopologyEdge : public TopologyFeature
{
public:
    TopologyEdge(ID id);

    std::vector<ID>& getVertexIds();
    const std::vector<ID>& getVertexIds() const;

    std::vector<ID>& getFaceIds();
    const std::vector<ID>& getFaceIds() const;

    std::vector<ID>& getCellIds();
    const std::vector<ID>& getCellIds() const;

    ID getOtherFaceId(ID faceId);

    // TopologyFeature interface
public:
    virtual Type getType() const;

private:
    std::vector<ID> mVertices;
    std::vector<ID> mFaces;
    std::vector<ID> mCells;
};

#endif // TOPOLOGYEDGE_H
