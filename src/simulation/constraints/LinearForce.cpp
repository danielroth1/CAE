#include "LinearForce.h"

#include "ConstraintVisitor.h"

using namespace Eigen;

LinearForce::LinearForce(
        SimulationPointRef source,
        SimulationPointRef target,
        double strength)
    : mSource(source)
    , mTarget(target)
    , mStrength(strength)
{

}

LinearForce::~LinearForce()
{
}

const SimulationPointRef& LinearForce::getTargetVector() const
{
    return mTarget;
}

const SimulationPointRef& LinearForce::getSourceVector() const
{
    return mSource;
}

double LinearForce::getStrength() const
{
    return mStrength;
}

void LinearForce::setTargetVector(SimulationPointRef target)
{
    mTarget = target;
}

void LinearForce::setSourceVector(SimulationPointRef source)
{
    mSource = source;
}

void LinearForce::setStrength(double strength)
{
    mStrength = strength;
}

void LinearForce::accept(ConstraintVisitor& cv)
{
    cv.visit(this);
}

bool LinearForce::references(Constraint* /*c*/)
{
    return false;
}

bool LinearForce::references(SimulationObject* so)
{
    return so == mSource.getSimulationObject() ||
            so == mTarget.getSimulationObject();
}
