#include "Polygon2DDataWS.h"

Polygon2DDataWS::Polygon2DDataWS(const Faces& faces, ID nVertices)
    : Polygon2DData (faces, nVertices)
{

}

Polygon2DDataWS::Polygon2DDataWS(const Polygon2DTopology& topology)
    : Polygon2DData(topology)
{

}

Polygon2DDataWS::~Polygon2DDataWS()
{

}

BSWSVectors::Type Polygon2DDataWS::getPositionType()
{
    return BSWSVectors::Type::WORLD_SPACE;
}
