#include "Polygon3DData.h"
#include "Polygon3DTopology.h"


Polygon3DData::Polygon3DData(
        const std::vector<unsigned int>& outerVertexIds,
        const Edges& edges,
        const Edges& outerEdges,
        const Faces& faces,
        const Faces& outerFaces,
        const Cells& cells)
{
    mTopology = std::make_unique<Polygon3DTopology>(
                outerVertexIds,
                edges,
                outerEdges,
                faces,
                outerFaces,
                cells);
}

Polygon3DData::~Polygon3DData()
{

}

std::vector<unsigned int>& Polygon3DData::getOuterVertexIds()
{
    return mTopology->getOuterVertexIds();
}

void Polygon3DData::setOuterVertexIds(
        const std::vector<unsigned int>& outerVertexIds)
{
    mTopology->setOuterVertexIds(outerVertexIds);
}

Edges& Polygon3DData::getEdges()
{
    return mTopology->getEdges();
}

void Polygon3DData::setEdges(const Edges& edges)
{
    mTopology->setEdges(edges);
}

Edges& Polygon3DData::getOuterEdges()
{
    return mTopology->getOuterEdges();
}

void Polygon3DData::setOuterEdges(const Edges& outerEdges)
{
    mTopology->setOuterEdges(outerEdges);
}

Faces& Polygon3DData::getFaces()
{
    return mTopology->getFaces();
}

void Polygon3DData::setFaces(const Faces& faces)
{
    mTopology->setFaces(faces);
}

Faces& Polygon3DData::getOuterFaces()
{
    return mTopology->getOuterFaces();
}

void Polygon3DData::setOuterFaces(const Faces& outerFaces)
{
    mTopology->setOuterFaces(outerFaces);
}

Cells& Polygon3DData::getCells()
{
    return mTopology->getCells();
}

void Polygon3DData::setCells(const Cells& cells)
{
    mTopology->setCells(cells);
}

Polygon::Type Polygon3DData::getType() const
{
    return Polygon::Type::THREE_D;
}
