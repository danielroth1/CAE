#include "SimulationPoint.h"

#include <simulation/SimulationObjectVisitor.h>

#include <scene/data/geometric/GeometricPoint.h>


SimulationPoint::SimulationPoint(
        Domain* domain,
        const std::shared_ptr<GeometricPoint> point)
    : SimulationObject (domain)
    , mPoint(point)
{

}

SimulationObject::Type SimulationPoint::getType() const
{
    return SimulationObject::Type::SIMULATION_POINT;
}

void SimulationPoint::accept(SimulationObjectVisitor& visitor)
{
    visitor.visit(*this);
}

void SimulationPoint::applyImpulse(
        SimulationPointRef& /*ref*/, const Vector& /*impulse*/)
{

}

void SimulationPoint::applyForce(
        SimulationPointRef& /*ref*/, const Vector& /*force*/)
{

}

Eigen::Vector& SimulationPoint::getPosition(size_t /*id*/)
{
    return mPoint->getPosition(0);
}

void SimulationPoint::setPosition(Eigen::Vector v, ID /*id*/)
{
    mPoint->setPosition(v);
}

void SimulationPoint::addToPosition(Eigen::Vector v, ID /*id*/)
{
    mPoint->setPosition(mPoint->getPosition() + v);
}

void SimulationPoint::integratePositions(double /*stepSize*/)
{

}

void SimulationPoint::revertPositions()
{

}

size_t SimulationPoint::getSize()
{
    return 1;
}

GeometricData* SimulationPoint::getGeometricData()
{
    return mPoint.get();
}
