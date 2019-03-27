#include "SimulationObjectFactory.h"

#include <scene/data/GeometricData.h>
#include <scene/data/GeometricDataVisitor.h>

#include <scene/data/geometric/GeometricPoint.h>
#include <scene/data/geometric/Polygon2D.h>
#include <scene/data/geometric/Polygon3D.h>

#include <simulation/fem/FEMObject.h>
#include <simulation/fem/SimulationPoint.h>

#include <simulation/rigid/RigidBody.h>

SimulationObjectFactory::SimulationObjectFactory()
{

}

std::vector<SimulationObject::Type>
SimulationObjectFactory::getPossibleSimulationObjectTypes(
        GeometricData& geometricData)
{
    class TypeIdentifierVisitor : public GeometricDataVisitor
    {
    public:
        virtual void visit(Polygon2D& /*polygon2D*/)
        {
        }

        virtual void visit(Polygon3D& /*polygon3D*/)
        {
            types.push_back(SimulationObject::Type::FEM_OBJECT);
        }

        virtual void visit(GeometricPoint& /*point*/)
        {
            types.push_back(SimulationObject::Type::SIMULATION_POINT);
        }

        std::vector<SimulationObject::Type> types;
    } visitor;

    geometricData.accept(visitor);
    return visitor.types;
}

SimulationPoint*
SimulationObjectFactory::createSimulationPoint(
        Domain* domain,
        GeometricPoint* point)
{
    return new SimulationPoint(domain, *point);
}

FEMObject*
SimulationObjectFactory::createFEMObject(
        Domain* domain,
        std::shared_ptr<Polygon3D> poly3)
{
    // TODO: ownership of poly3 should be passed here!
    return new FEMObject(domain, poly3);
}

RigidBody* SimulationObjectFactory::createRigidBody(
        Domain* domain,
        std::shared_ptr<Polygon2D> poly2,
        double mass)
{
    return new RigidBody(domain, poly2, poly2->getPositions(), mass);
}

RigidBody* SimulationObjectFactory::createRigidBody(
        Domain* domain,
        std::shared_ptr<Polygon3D> poly3,
        double mass)
{
    return new RigidBody(domain, poly3, poly3->getPositions(), mass);
}
