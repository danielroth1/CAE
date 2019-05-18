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
