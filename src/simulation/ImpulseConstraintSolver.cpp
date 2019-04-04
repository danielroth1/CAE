#include "ImpulseConstraintSolver.h"

#include <simulation/collision_detection/narrow/Collision.h>
#include <iostream>
#include <simulation/fem/FEMObject.h>
#include <simulation/rigid/RigidBody.h>
#include <simulation/constraints/CollisionConstraint.h>

using namespace Eigen;

ImpulseConstraintSolver::ImpulseConstraintSolver()
{

}

ImpulseConstraintSolver::~ImpulseConstraintSolver()
{

}

void ImpulseConstraintSolver::initialize(
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

void ImpulseConstraintSolver::solveConstraints(int maxIterations, double maxConstraintError)
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

bool ImpulseConstraintSolver::solveConstraint(
        CollisionConstraint& cc, double maxConstraintError)
{

    // speed at
    Collision& c = cc.getCollision();

    Eigen::Vector p1 = calculateRelativePoint(c.getSimulationObjectA(), c.getPointA());
    Eigen::Vector p2 = calculateRelativePoint(c.getSimulationObjectB(), c.getPointB());

    Vector uRel = calculateRelativeSpeed(
                calculateSpeed(c.getSimulationObjectA(), p1, c.getVertexIndexA()),
                calculateSpeed(c.getSimulationObjectB(), p2, c.getVertexIndexB()),
                c.getNormal());

    Vector deltaUNormalRel = cc.getTargetUNormalRel() - uRel;
    if (deltaUNormalRel.norm() < maxConstraintError)
    {
        return true;
    }
    Vector impulse = cc.getImpulseFactor() * deltaUNormalRel;
    if (c.getNormal().dot(cc.getSumOfAllAppliedImpulses() + impulse) < 0)
    {
        impulse = -cc.getSumOfAllAppliedImpulses();
    }
    cc.setSumOfAllAppliedImpulses(cc.getSumOfAllAppliedImpulses() + impulse);

    applyImpulse(c.getSimulationObjectA(), impulse, p1, c.getVertexIndexA());
    applyImpulse(c.getSimulationObjectB(), -impulse, p2, c.getVertexIndexB());

    return false;
}

bool ImpulseConstraintSolver::solveConstraint(
        BallJoint& ballJoint, double maxConstraintError)
{
    // TODO: implement this
}

Matrix3d ImpulseConstraintSolver::calculateK(
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

Vector ImpulseConstraintSolver::calculateRelativeSpeed(
        const Vector& relativeSpeedA,
        const Vector& relativeSpeedB,
        const Vector& normal)
{
    return (relativeSpeedA - relativeSpeedB).dot(normal) * normal;
}

Vector ImpulseConstraintSolver::calculateSpeed(
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

void ImpulseConstraintSolver::applyImpulse(
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

Vector ImpulseConstraintSolver::calculateRelativePoint(SimulationObject* so, const Vector& pointGlobal)
{
    if (so->getType() == SimulationObject::Type::RIGID_BODY)
    {
        RigidBody* rb = static_cast<RigidBody*>(so);
        return pointGlobal - rb->getCenterOfMass();
    }
    return pointGlobal;
}
