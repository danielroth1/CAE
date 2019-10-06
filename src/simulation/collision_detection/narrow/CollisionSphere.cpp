#include "CollisionObjectVisitor.h"
#include "CollisionSphere.h"

#include <scene/data/references/GeometricPointRef.h>
#include <scene/data/references/GeometricPointRefVisitor.h>
#include <scene/data/references/GeometricVertexRef.h>
#include <scene/data/references/PolygonVectorRef.h>

CollisionSphere::CollisionSphere(
        SimulationPointRef pointRef,
        double radius,
        const std::shared_ptr<TopologyFeature>& topologyFeature)
    : mPointRef(pointRef)
    , mRadius(radius)
    , mFeature(topologyFeature)
{
    mPointRef.setUpdatePolicy(SimulationPointRef::UpdatePolicy::ON_UPDATE_CALL);
    mPointRef.update();
}

ID CollisionSphere::getVertexIndex()
{
    class ExtractIndexVisitor : public GeometricPointRefVisitor
    {
    public:

        virtual void visit(GeometricVertexRef& ref)
        {
           index = static_cast<ID>(ref.getIndex());
        }

        virtual void visit(PolygonVectorRef& /*ref*/)
        {
            index = -1UL;
        }

        ID index;
    } visitor;

    mPointRef.getGeometricPointRef()->accept(visitor);
    return visitor.index;
}

void CollisionSphere::update()
{
    mPointRef.update();
}

void CollisionSphere::updatePrevious()
{
    mPointRef.updatePrevious();
}

CollisionObject::Type CollisionSphere::getType() const
{
    return CollisionObject::Type::SPHERE;
}

Eigen::Vector CollisionSphere::getPosition()
{
    return mPointRef.getPoint();
}
