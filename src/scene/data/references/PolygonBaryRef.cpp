#include "GeometricPointRefVisitor.h"
#include "PolygonBaryRef.h"

#include <scene/data/geometric/Polygon3D.h>
#include <scene/data/geometric/Polygon3DTopology.h>


PolygonBaryRef::PolygonBaryRef(
        Polygon3D* polygon, const std::array<double, 4>& bary, ID elementId)
    : GeometricPointRef(polygon, GeometricPointRef::Type::POLYGON_BARY)
    , mPolygon(polygon)
    , mBary(bary)
    , mElementId(elementId)
{

}

Vector PolygonBaryRef::getPoint() const
{
    Polygon3DTopology& topo = mPolygon->getTopology3D();
    TopologyCell& cell = topo.getCells()[mElementId];
    const std::array<unsigned int, 4>& vIds = cell.getVertexIds();
    return mBary[0] * mPolygon->getPosition(vIds[0]) +
            mBary[1] * mPolygon->getPosition(vIds[1]) +
            mBary[2] * mPolygon->getPosition(vIds[2]) +
            mBary[3] * mPolygon->getPosition(vIds[3]);
}

ID PolygonBaryRef::getIndex() const
{
    return mPolygon->getTopology3D().getCells()[mElementId].getVertexIds()[0];
}

GeometricPointRef* PolygonBaryRef::clone()
{
    return new PolygonBaryRef(mPolygon, mBary, mElementId);
}

void PolygonBaryRef::accept(GeometricPointRefVisitor& visitor)
{
    visitor.visit(*this);
}
