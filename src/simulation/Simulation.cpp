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

#include <scene/data/geometric/Polygon.h>

using namespace Eigen;


Simulation::Simulation(
        Domain* domain,
        double timeStep,
        std::shared_ptr<CollisionManager> collisionManager)
    : mDomain(domain)
    , mTimeStep(timeStep)
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

void Simulation::addLinearForce(std::shared_ptr<LinearForce> linearForce)
{
    auto it = std::find(mLinearForces.begin(), mLinearForces.end(), linearForce);
    if (it == mLinearForces.end())
    {
        mLinearForces.push_back(linearForce);
    }
}

bool Simulation::removeLinearForce(LinearForce* linearForce)
{
    auto it = std::find_if(
                mLinearForces.begin(),
                mLinearForces.end(),
                [&linearForce](const std::shared_ptr<LinearForce>& lf)
    {
        return lf.get() == linearForce;
    }); // lambda expression to compare LinearForce* with std::shared_ptr<LinearForce>
    if (it != mLinearForces.end())
    {
        mLinearForces.erase(it);
        return true;
    }
    return false;
}

std::vector<std::shared_ptr<LinearForce>>& Simulation::getLinearForces()
{
    return mLinearForces;
}

void Simulation::setTimeStep(double timeStep)
{
    mTimeStep = timeStep;
}

double Simulation::getTimeStep() const
{
    return mTimeStep;
}

Domain* Simulation::getDomain()
{
    return mDomain;
}
