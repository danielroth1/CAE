#include "RigidBody.h"
#include "RigidCollisionSolver.h"
#include "RigidSimulation.h"

#include <simulation/collision_detection/narrow/Collision.h>
#include <iostream>
#include <simulation/fem/FEMObject.h>

using namespace Eigen;

RigidCollisionSolver::RigidCollisionSolver()
{

}

RigidCollisionSolver::~RigidCollisionSolver()
{

}

void RigidCollisionSolver::initialize(
        std::vector<Collision>& collisions,
        double stepSize,
        double restitution,
        double maxCollisionDistance)
{
    // calculate K, target u rels
    mCollisionConstraints.clear();
    mCollisionConstraints.reserve(collisions.size());

    for (size_t i = 0; i < collisions.size(); ++i)
    {
        Collision& c = collisions[i];

        Eigen::Vector p1 = calculateRelativePoint(c.getSimulationObjectA(), c.getPointA());
        Eigen::Vector p2 = calculateRelativePoint(c.getSimulationObjectB(), c.getPointB());

        Eigen::Vector targetUNormalRel = -restitution * calculateRelativeSpeed(
                    calculateSpeed(c.getSimulationObjectA(), p1, c.getVertexIndexA()),
                    calculateSpeed(c.getSimulationObjectB(), p2, c.getVertexIndexB()),
                    c.getNormal());

        double impulseFactor = 1 / (c.getNormal().transpose() *
                                    (calculateK(c.getSimulationObjectA(), p1, c.getVertexIndexA()) +
                                     calculateK(c.getSimulationObjectB(), p2, c.getVertexIndexB()))
                             * c.getNormal());

        mCollisionConstraints.push_back(
                    CollisionConstraint(c, targetUNormalRel,
                                        Vector::Zero(), impulseFactor));
    }
}

void RigidCollisionSolver::solveConstraints(int maxIterations, double maxConstraintError)
{
    size_t validConstraints = 0;

    int iterCount = 0;
    for (int iter = 0; iter < maxIterations; ++iter)
    {
        ++iterCount;
        // iterate collisions for as long as each u rel is equal to target u rel
        for (size_t i = 0; i < mCollisionConstraints.size(); ++i)
        {
            if (validConstraints == mCollisionConstraints.size())
                break;

            if (solveConstraint(mCollisionConstraints[i], maxConstraintError))
            {
                // if constraint is valid, nothing to do
                ++validConstraints;
            }
            else
            {
                // if not, this corrected coinstraint will be the only one
                // we can be sure about to be valid since the correction
                // can make any other constraint invalid again.
                validConstraints = 1;
            }
        }
        if (validConstraints == mCollisionConstraints.size())
            break;
    }

    std::cout << "took " << iterCount << " iterations\n";
}

bool RigidCollisionSolver::solveConstraint(CollisionConstraint& cc, double maxConstraintError)
{

    // speed at
    Collision& c = cc.collision;

    Eigen::Vector p1 = calculateRelativePoint(c.getSimulationObjectA(), c.getPointA());
    Eigen::Vector p2 = calculateRelativePoint(c.getSimulationObjectB(), c.getPointB());

    Vector uRel = calculateRelativeSpeed(
                calculateSpeed(c.getSimulationObjectA(), p1, c.getVertexIndexA()),
                calculateSpeed(c.getSimulationObjectB(), p2, c.getVertexIndexB()),
                c.getNormal());

    Vector deltaUNormalRel = cc.targetUNormalRel - uRel;
    if (deltaUNormalRel.norm() < maxConstraintError)
    {
        return true;
    }
    Vector impulse = cc.impulseFactor * deltaUNormalRel;
    if (c.getNormal().dot(cc.sumOfAllAppliedImpulses + impulse) < 0/* && cc.impulseApplied*/)
    {
        impulse = -cc.sumOfAllAppliedImpulses;
    }
    cc.impulseApplied = true;
    cc.sumOfAllAppliedImpulses += impulse;

    applyImpulse(c.getSimulationObjectA(), impulse, p1, c.getVertexIndexA());
    applyImpulse(c.getSimulationObjectB(), -impulse, p2, c.getVertexIndexB());

    return false;
}

Matrix3d RigidCollisionSolver::calculateK(
        SimulationObject* so,
        const Vector& point,
        const ID vertexIndex)
{
    switch(so->getType())
    {
    case SimulationObject::Type::RIGID_BODY:
    {
        RigidBody* rb = static_cast<RigidBody*>(so);
        return rb->calculateK(point, point);
    }
    case SimulationObject::Type::FEM_OBJECT:
    {
        FEMObject* femObj = static_cast<FEMObject*>(so);
        return 1 / femObj->getMass(vertexIndex) * Eigen::Matrix3d::Identity();
    }
    case SimulationObject::Type::SIMULATION_POINT:
    {
        break;
    }
    }
    return Eigen::Matrix3d::Identity();
}

Vector RigidCollisionSolver::calculateRelativeSpeed(
        const Vector& relativeSpeedA,
        const Vector& relativeSpeedB,
        const Vector& normal)
{
    return (relativeSpeedA - relativeSpeedB).dot(normal) * normal;
}

Vector RigidCollisionSolver::calculateSpeed(
        SimulationObject* so,
        const Vector& point,
        const ID vertexIndex)
{
    switch(so->getType())
    {
    case SimulationObject::Type::RIGID_BODY:
    {
        RigidBody* rb = static_cast<RigidBody*>(so);
        return rb->calculateSpeedAt(point);
    }
    case SimulationObject::Type::FEM_OBJECT:
    {
        FEMObject* femObj = static_cast<FEMObject*>(so);
        return femObj->getVelocities()[vertexIndex];
    }
    case SimulationObject::Type::SIMULATION_POINT:
        break;
    }
    return Vector::Zero();
}

void RigidCollisionSolver::applyImpulse(
        SimulationObject* so,
        const Vector& impulse,
        const Vector& point,
        const ID vertexIndex)
{
    switch(so->getType())
    {
    case SimulationObject::Type::RIGID_BODY:
    {
        RigidBody* rb = static_cast<RigidBody*>(so);
        rb->applyImpulse(point, impulse);
        break;
    }
    case SimulationObject::Type::FEM_OBJECT:
    {
        FEMObject* femObj = static_cast<FEMObject*>(so);
        femObj->applyImpulse(vertexIndex, impulse);
        break;
    }
    case SimulationObject::Type::SIMULATION_POINT:
        break;
    }
}

Vector RigidCollisionSolver::calculateRelativePoint(SimulationObject* so, const Vector& pointGlobal)
{
    if (so->getType() == SimulationObject::Type::RIGID_BODY)
    {
        RigidBody* rb = static_cast<RigidBody*>(so);
        return pointGlobal - rb->getCenterOfMass();
    }
    return pointGlobal;
}

RigidCollisionSolver::CollisionConstraint::CollisionConstraint(
        Collision& _collision,
        Eigen::Vector _targetUNormalRel,
        Eigen::Vector _sumOfAllAppliedImpulses,
        double _impulseFactor)
    : collision(_collision)
    , targetUNormalRel(_targetUNormalRel)
    , sumOfAllAppliedImpulses(_sumOfAllAppliedImpulses)
    , impulseFactor(_impulseFactor)
    , impulseApplied(false)
{

}
