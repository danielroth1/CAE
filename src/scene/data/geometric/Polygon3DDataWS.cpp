#include "Polygon3DDataWS.h"
#include "Polygon3DTopology.h"


Polygon3DDataWS::Polygon3DDataWS(
        const std::vector<unsigned int>& outerVertexIds,
        const Edges& edges,
        const Edges& outerEdges,
        const Faces& faces,
        const Faces& outerFaces,
        const Cells& cells)
    : Polygon3DData(outerVertexIds, edges, outerEdges, faces, outerFaces, cells)
{
}

Polygon3DDataWS::~Polygon3DDataWS()
{

}
