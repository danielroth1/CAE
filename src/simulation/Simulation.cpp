#include "Simulation.h"
#include "SimulationObject.h"
#include "SimulationObjectVisitor.h"
#include "ui/UniqueVertex.h"
#include "simulation/forces/LinearForce.h"

#include <scene/data/references/GeometricPointRefVisitor.h>
#include <scene/data/references/GeometricVertexRef.h>
#include <scene/data/references/PolygonVectorRef.h>

#include <simulation/fem/SimulationPoint.h>

#include <simulation/rigid/RigidBody.h>

#include <scene/data/geometric/AbstractPolygon.h>

using namespace Eigen;


Simulation::Simulation(
        Domain* domain,
        std::shared_ptr<CollisionManager> collisionManager)
    : mDomain(domain)
    , mCollisionManager(collisionManager)
{

}

Simulation::~Simulation()
{
}

void Simulation::actExternalForce(SimulationPointRef* ref, Vector force)
{
    ref->getSimulationObject()->applyForce(*ref, force);
}

Domain* Simulation::getDomain()
{
    return mDomain;
}
