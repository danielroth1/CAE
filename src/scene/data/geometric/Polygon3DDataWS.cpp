#include "Polygon3DDataWS.h"
#include "Polygon3DTopology.h"


Polygon3DDataWS::Polygon3DDataWS(
        const Faces& faces,
        const Faces& outerFaces,
        const Cells& cells,
        ID nVertices)
    : Polygon3DData(faces, outerFaces, cells, nVertices)
{
}

Polygon3DDataWS::~Polygon3DDataWS()
{

}
