#ifndef TOPOLOGYEDGE_H
#define TOPOLOGYEDGE_H

#include "TopologyFeature.h"

#include <data_structures/DataStructures.h>

class TopologyEdge : public TopologyFeature
{
public:
    TopologyEdge(ID id, ID geometryId);

    Edge& getVertexIds()
    {
        return mVertices;
    }

    const Edge& getVertexIds() const
    {
        return mVertices;
    }

    std::vector<ID>& getFaceIds()
    {
        return mFaces;
    }

    const std::vector<ID>& getFaceIds() const
    {
        return mFaces;
    }

    std::vector<ID>& getCellIds()
    {
        return mCells;
    }

    const std::vector<ID>& getCellIds() const
    {
        return mCells;
    }

    ID getOtherFaceId(ID faceId)
    {
        return faceId == mFaces[0] ? mFaces[1] : mFaces[0];
    }

    // TopologyFeature interface
public:
    virtual Type getType() const;

private:
    Edge mVertices;
    std::vector<ID> mFaces;
    std::vector<ID> mCells;
};

#endif // TOPOLOGYEDGE_H
