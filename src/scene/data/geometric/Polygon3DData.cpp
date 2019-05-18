#include "Polygon3DData.h"
#include "Polygon3DTopology.h"


Polygon3DData::Polygon3DData(const Polygon3DTopology& topology)
{
    mTopology = std::make_unique<Polygon3DTopology>(topology);
}

Polygon3DData::~Polygon3DData()
{

}

Polygon3DTopology& Polygon3DData::getTopology()
{
    return *mTopology.get();
}

Polygon::DimensionType Polygon3DData::getDimensionType() const
{
    return Polygon::DimensionType::THREE_D;
}
