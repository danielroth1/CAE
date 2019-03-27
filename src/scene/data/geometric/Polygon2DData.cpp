#include "Polygon2DData.h"
#include "Polygon2DTopology.h"

Polygon2DData::Polygon2DData(
        const Faces& faces,
        const Edges& edges)
{
    mTopology = std::make_unique<Polygon2DTopology>(
                faces, edges);
}

Polygon2DData::~Polygon2DData()
{

}

Faces& Polygon2DData::getFaces()
{
    return mTopology->getFaces();
}

Edges& Polygon2DData::getEdges()
{
    return mTopology->getEdges();
}

Polygon::Type Polygon2DData::getType() const
{
    return Polygon::Type::TWO_D;
}
