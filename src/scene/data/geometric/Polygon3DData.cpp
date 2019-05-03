#include "Polygon3DData.h"
#include "Polygon3DTopology.h"


Polygon3DData::Polygon3DData(
        const Faces& faces,
        const Faces& outerFaces,
        const Cells& cells,
        ID nVertices)
{
    mTopology = std::make_unique<Polygon3DTopology>(
                faces, outerFaces, cells, nVertices);
}

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

Polygon::Type Polygon3DData::getType() const
{
    return Polygon::Type::THREE_D;
}
