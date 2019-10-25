#ifndef SIMULATIONOBJECTFACTORY_H
#define SIMULATIONOBJECTFACTORY_H

#include "SimulationObject.h"

#include <vector>

class Domain;
class FEMObject;
class GeometricData;
class GeometricPoint;
class Polygon2D;
class Polygon3D;
class RigidBody;
class SimulationPoint;

// This class an interface between GeometricData
// and SimulationObjects and is therefore one of
// the few classes that depend on both geometry
// and simulation. This is not only a factory
// but features other methods that describe the
// relation between GeometricData and SimulationObject,
// e.g. getPossibleSimulationObjectTypes(GeometriData&)
class SimulationObjectFactory
{
public:
    SimulationObjectFactory();

    // Returns all possible simulation objects that can be
    // created from the given GeometricData. The vector is
    // sorted.
    static std::vector<SimulationObject::Type>
    getPossibleSimulationObjectTypes(GeometricData& geometricData);

    static SimulationPoint* createSimulationPoint(
            Domain* domain,
            const std::shared_ptr<GeometricPoint>& point);

    static FEMObject* createFEMObject(Domain* domain, std::shared_ptr<Polygon3D> poly3, double mass);

    static RigidBody* createRigidBody(Domain* domain, std::shared_ptr<Polygon2D> poly2, double mass);

    static RigidBody* createRigidBody(Domain* domain, std::shared_ptr<Polygon3D> poly3, double mass);
};

#endif // SIMULATIONOBJECTFACTORY_H
