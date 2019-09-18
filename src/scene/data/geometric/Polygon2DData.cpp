#include "Polygon2DData.h"
#include "Polygon2DTopology.h"


Polygon2DData::Polygon2DData(const Polygon2DTopology& topology)
{
    mTopology = std::make_unique<Polygon2DTopology>(topology);
}

Polygon2DData::~Polygon2DData()
{

}

Polygon2DTopology& Polygon2DData::getTopology()
{
    return *mTopology.get();
}

Polygon::DimensionType Polygon2DData::getDimensionType() const
{
    return Polygon::DimensionType::TWO_D;
}

void Polygon2DData::removeVector(ID index)
{
    std::vector<ID> indices;
    indices.push_back(index);
    mTopology->removeVertices(indices);
}

void Polygon2DData::removeVectors(std::vector<ID>& indices)
{
    mTopology->removeVertices(indices);
}
