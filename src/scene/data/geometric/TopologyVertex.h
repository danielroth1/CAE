#ifndef TOPOLOGYVERTEX_H
#define TOPOLOGYVERTEX_H

#include <data_structures/DataStructures.h>

class TopologyVertex
{
public:
    TopologyVertex();

    std::vector<ID>& getEdgeIds();
    const std::vector<ID>& getEdgeIds() const;
    std::vector<ID>& getFaceIds();
    const std::vector<ID>& getFaceIds() const;

private:
    std::vector<ID> mEdges;
    std::vector<ID> mFaces;
};

#endif // TOPOLOGYVERTEX_H
