#include "Polygon3DData.h"
#include "Polygon3DTopology.h"


Polygon3DData::Polygon3DData(const std::shared_ptr<Polygon3DTopology>& topology)
    : mTopology(topology)
{
}

Polygon3DData::~Polygon3DData()
{

}

const std::shared_ptr<Polygon3DTopology>& Polygon3DData::getTopology() const
{
    return mTopology;
}

Polygon::DimensionType Polygon3DData::getDimensionType() const
{
    return Polygon::DimensionType::THREE_D;
}

void Polygon3DData::removeVector(ID index)
{
    std::vector<ID> indices;
    indices.push_back(index);
    mTopology->removeVertices(indices);
}

void Polygon3DData::removeVectors(std::vector<ID>& indices)
{
    mTopology->removeVertices(indices);
}
