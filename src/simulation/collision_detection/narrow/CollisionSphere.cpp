#include "CollisionObjectVisitor.h"
#include "CollisionSphere.h"

#include <scene/data/references/GeometricPointRef.h>
#include <scene/data/references/GeometricPointRefVisitor.h>
#include <scene/data/references/GeometricVertexRef.h>
#include <scene/data/references/PolygonVectorRef.h>

CollisionSphere::CollisionSphere(
        SimulationPointRef pointRef,
        double radius)
    : mPointRef(pointRef)
    , mRadius(radius)
{

}

double CollisionSphere::getRadius() const
{
    return mRadius;
}

void CollisionSphere::setRadius(double radius)
{
    mRadius = radius;
}

SimulationPointRef& CollisionSphere::getPointRef()
{
    return mPointRef;
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

void CollisionSphere::accept(CollisionObjectVisitor& visitor)
{
    visitor.visit(this);
}

Eigen::Vector CollisionSphere::getPosition() const
{
    return mPointRef.getPoint();
}
