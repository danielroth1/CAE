#ifndef TOPOLOGYCELL_H
#define TOPOLOGYCELL_H

#include "TopologyFeature.h"

#include <array>
#include <vector>

class TopologyCell : public TopologyFeature
{
public:
    TopologyCell(ID id, ID geometryId);

    std::array<unsigned int, 4>& getVertexIds();
    const std::array<unsigned int, 4>& getVertexIds() const;

    std::array<unsigned int, 6>& getEdgeIds();
    const std::array<unsigned int, 6>& getEdgeIds() const;

    std::vector<unsigned int>& getFaceIds();
    const std::vector<unsigned int>& getFaceIds() const;

    // TopologyFeature interface
public:
    virtual Type getType() const;

private:

    std::array<unsigned int, 4> mVertexIds;
    std::array<unsigned int, 6> mEdgeIds;
    std::vector<unsigned int> mFaceIds;
};

#endif // TOPOLOGYCELL_H
