#include "LinearForce.h"

#include <simulation/ImpulseConstraintSolver.h>

using namespace Eigen;

LinearForce::LinearForce(
        SimulationPointRef source,
        SimulationPointRef target,
        double strength,
        double damping,
        double length)
    : mSource(source)
    , mTarget(target)
    , mStrength(strength)
    , mDamping(damping)
    , mLength(length)
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
    Eigen::Vector force;

    // spring
    if (mLength > 1e-10)
    {
        Vector direction = mTarget.getPoint() - mSource.getPoint();
        if (direction.norm() < 1e-10)
            direction = Eigen::Vector(1.0, 0.0, 0.0);
        else
            direction.normalize();

        force = mStrength * (mTarget.getPoint() - mSource.getPoint() -
                             mLength * direction);
    }
    else
    {
        force = mStrength * (mTarget.getPoint() - mSource.getPoint());
    }

    // damping
    if (mDamping > 1e-10)
    {
        Vector direction = mTarget.getPoint() - mSource.getPoint();
        if (direction.norm() > 1e-10)
        {
            direction.normalize();

            // -mDamoing * (v2 - v1).dot(normal) * direction
            // how to obtain v?
            Eigen::Vector v1 = ImpulseConstraintSolver::calculateSpeed(
                        mSource.getSimulationObject().get(),
                        ImpulseConstraintSolver::calculateRelativePoint(
                            mSource.getSimulationObject().get(), mSource.getPoint()),
                        mSource.getIndex());

            Eigen::Vector v2 = ImpulseConstraintSolver::calculateSpeed(
                        mTarget.getSimulationObject().get(),
                        ImpulseConstraintSolver::calculateRelativePoint(
                            mTarget.getSimulationObject().get(), mTarget.getPoint()),
                        mTarget.getIndex());

            force += mDamping * (v2 - v1).dot(direction) * direction;
        }
    }

    mSource.getSimulationObject()->applyForce(mSource, force);
    mTarget.getSimulationObject()->applyForce(mTarget, -force);
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
    return so == mSource.getSimulationObject().get() ||
            so == mTarget.getSimulationObject().get();
}
