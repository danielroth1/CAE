#include "LinearForce.h"

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

void LinearForce::applyForce()
{
    // TODO: this only workd at the start of the simulation when
    // mTarget.getPoint() and mSource.getPoint() actually return the
    // correct points. At every other point in the simulation, if the
    // positoin of the simulated object was updated, but not the geometry,
    // this method will use the old, now wrong, position of the geometry
    // again. How to obtain the actual position? Maybe overwrite the
    // SimulationPointRef::getPoint() method, so that these kind of errors
    // won't happen in the future?
    Eigen::Vector force =
            mStrength * (mTarget.getPoint() - mSource.getPoint());
    mSource.getSimulationObject()->applyForce(mSource, force);
    mTarget.getSimulationObject()->applyForce(mSource, -force);
}

SimulationPointRef& LinearForce::getTargetVector()
{
    return mTarget;
}

SimulationPointRef& LinearForce::getSourceVector()
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

bool LinearForce::references(SimulationObject* so)
{
    return so == mSource.getSimulationObject() ||
            so == mTarget.getSimulationObject();
}
