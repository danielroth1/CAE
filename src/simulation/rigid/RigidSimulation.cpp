#include "RigidBody.h"
#include "RigidSimulation.h"

#include <simulation/collision_detection/CollisionManager.h>
#include <simulation/forces/LinearForce.h>
#include <simulation/SimulationObjectVisitor.h>
#include <scene/data/GeometricData.h>
#include <scene/data/geometric/Polygon.h>

#include <iostream>


using namespace Eigen;

RigidSimulation::RigidSimulation(
        Domain* domain,
        double timeStep,
        std::shared_ptr<CollisionManager> collisionManager)
    : Simulation(domain, timeStep, collisionManager)
{
}

RigidSimulation::~RigidSimulation()
{

}

void RigidSimulation::addRigidBody(std::shared_ptr<RigidBody> rigidBody)
{
    mRigidBodies.push_back(rigidBody);

    // TODO_RIGIDSIMULATION: implement this
}

bool RigidSimulation::removeRigidBody(RigidBody* rigidBody)
{
    auto it = std::find_if(
                mRigidBodies.begin(),
                mRigidBodies.end(),
                [&](const std::shared_ptr<RigidBody>& p) {
        return p.get() == rigidBody;
    });
    if (it != mRigidBodies.end())
    {
        mRigidBodies.erase(it);
        return true;
    }
    return false;
}

void RigidSimulation::initializeStep()
{
    for (std::shared_ptr<RigidBody>& rb : mRigidBodies)
    {
        rb->update();
        rb->prepareNewStep();
    }

    // gravity
    for (std::shared_ptr<RigidBody>& rb : mRigidBodies)
    {
        rb->applyForce(Vector::Zero(), rb->getMass() * Vector(0, -9.81, 0));
    }
}

void RigidSimulation::solve()
{
    for (std::shared_ptr<RigidBody>& rb : mRigidBodies)
    {
        rb->solveExplicit(mTimeStep);
    }
}

void RigidSimulation::integratePositions()
{
    for (std::shared_ptr<RigidBody>& rb : mRigidBodies)
    {
        rb->integratePositions(mTimeStep);
    }
}

void RigidSimulation::revertPositions()
{
    for (std::shared_ptr<RigidBody>& rb : mRigidBodies)
    {
        rb->revertPositions();
    }
}

void RigidSimulation::applyDamping()
{
    for (std::shared_ptr<RigidBody>& rb : mRigidBodies)
    {
        rb->applyDamping();
    }
}

void RigidSimulation::publish()
{
    for (std::shared_ptr<RigidBody>& rb : mRigidBodies)
    {
        rb->updateGeometricData();
    }
}

void RigidSimulation::initialize()
{
}

void RigidSimulation::step()
{
    initializeStep();
    solve();
}

Vector RigidSimulation::calculateRelativeSpeed(
        RigidBody* body1, const Vector& r1,
        RigidBody* body2, const Vector& r2,
        const Eigen::Vector& normal)
{
    return (body1->calculateSpeedAt(r1) - body2->calculateSpeedAt(r2)).dot(normal) * normal;
}

Vector RigidSimulation::calculateCollisionCorrectionImpulse(
        RigidBody* body1, const Vector& r1,
        RigidBody* body2, const Vector& r2,
        const Eigen::Vector& targetURel,
        const Vector& normal)
{
    // 1 / (n^T K n) * Delta_u_rel
    Eigen::Matrix3d K = body1->calculateK(r1, r1) + body2->calculateK(r2, r2);
    double factor = 1 / (normal.transpose() * K * normal);
    // This is not correct. Here Deltae rel speed should be used
    // \Delta u_{rel, n} = u^c_{rel, n} - u_{rel, n}
    return factor * (targetURel - calculateRelativeSpeed(body1, r1, body2, r2, normal));
}

void RigidSimulation::solveExplicitly()
{

}
