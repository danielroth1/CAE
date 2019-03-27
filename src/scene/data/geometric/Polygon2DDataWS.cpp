#include "Polygon2DDataWS.h"

Polygon2DDataWS::Polygon2DDataWS(
        const Faces& faces,
        const Edges& edges)
    : Polygon2DData (faces, edges)
{

}

Polygon2DDataWS::~Polygon2DDataWS()
{

}

BSWSVectors::Type Polygon2DDataWS::getPositionType()
{
    return BSWSVectors::Type::WORLD_SPACE;
}
