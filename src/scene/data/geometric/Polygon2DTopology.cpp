#include "Polygon2DTopology.h"

Polygon2DTopology::Polygon2DTopology()
{

}

Polygon2DTopology::Polygon2DTopology(
        const Faces& faces,
        const Edges& edges)
    : mFaces(faces)
    , mEdges(edges)
{

}

void Polygon2DTopology::setFaces(const Faces& faces)
{
    mFaces = faces;
}

void Polygon2DTopology::setEdges(const Edges& edges)
{
    mEdges = edges;
}

Faces&Polygon2DTopology::getFaces()
{
    return mFaces;
}

Edges&Polygon2DTopology::getEdges()
{
    return mEdges;
}
