#include "ImpulseConstraintSolver.h"

#include <simulation/collision_detection/narrow/Collision.h>
#include <iostream>
#include <simulation/fem/FEMObject.h>
#include <simulation/rigid/RigidBody.h>
#include <simulation/constraints/BallJoint.h>
#include <simulation/constraints/CollisionConstraint.h>

using namespace Eigen;

ImpulseConstraintSolver::ImpulseConstraintSolver()
{

}

ImpulseConstraintSolver::~ImpulseConstraintSolver()
{

}

void ImpulseConstraintSolver::initializeNonCollisionConstraints(double stepSize)
{
    for (size_t i = 0; i < mConstraints.size(); ++i)
    {
        mConstraints[i]->initialize(stepSize);
    }
}

void ImpulseConstraintSolver::initializeCollisionConstraints(
        std::vector<Collision>& collisions,
        double restitution,
        double stepSize)
{
    // calculate K, target u rels
    mCollisionConstraints.clear();
    mCollisionConstraints.reserve(collisions.size());

    for (size_t i = 0; i < collisions.size(); ++i)
    {
        Collision& c = collisions[i];
        mCollisionConstraints.push_back(CollisionConstraint(c, restitution));
    }

    for (size_t i = 0; i < mCollisionConstraints.size(); ++i)
    {
        mCollisionConstraints[i].initialize(stepSize);
    }
}

void ImpulseConstraintSolver::solveConstraints(int maxIterations, double maxConstraintError)
{
    size_t validConstraints = 0;
    size_t totalConstraints = mConstraints.size() + mCollisionConstraints.size();

    int iterCount = 0;
    for (int iter = 0; iter < maxIterations; ++iter)
    {
        ++iterCount;

        // iterate non-collision constraints
        for (size_t i = 0; i < mConstraints.size(); ++i)
        {
            if (validConstraints == totalConstraints)
                break;

            if (mConstraints[i]->solve(maxConstraintError))
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

        // iterate collisions for as long as each u rel is equal to target u rel
        for (size_t i = 0; i < mCollisionConstraints.size(); ++i)
        {
            if (validConstraints == totalConstraints)
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
        if (validConstraints == totalConstraints)
            break;
    }

//    std::cout << "took " << iterCount << " iterations\n";
}

bool ImpulseConstraintSolver::solveConstraint(
        CollisionConstraint& cc, double maxConstraintError)
{
    return cc.solve(maxConstraintError);
}

bool ImpulseConstraintSolver::solveConstraint(
        BallJoint& ballJoint, double maxConstraintError)
{
    return ballJoint.solve(maxConstraintError);
}

Matrix3d ImpulseConstraintSolver::calculateK(
        SimulationObject* so,
        const Vector& point,
        const ID vertexIndex)
{
    switch(so->getType())
    {
    case SimulationObject::Type::FEM_OBJECT:
    {
        FEMObject* femObj = static_cast<FEMObject*>(so);
        return 1 / femObj->getMass(vertexIndex) * Eigen::Matrix3d::Identity();
    }
    case SimulationObject::Type::RIGID_BODY:
    {
        RigidBody* rb = static_cast<RigidBody*>(so);
        return rb->calculateK(point, point);
    }
    case SimulationObject::Type::SIMULATION_POINT:
    {
        break;
    }
    }
    return Eigen::Matrix3d::Identity();
}

Matrix3d ImpulseConstraintSolver::calculateK(SimulationPointRef& ref)
{
    switch(ref.getSimulationObject()->getType())
    {
    case SimulationObject::Type::FEM_OBJECT:
    {
        FEMObject* femObj = static_cast<FEMObject*>(ref.getSimulationObject());
        ID index = ref.getIndex();
        if (index != ILLEGAL_INDEX)
        {
            return 1 / femObj->getMass(index) * Eigen::Matrix3d::Identity();
        }
        break;
    }
    case SimulationObject::Type::RIGID_BODY:
    {
        RigidBody* rb = static_cast<RigidBody*>(ref.getSimulationObject());
        Eigen::Vector r = rb->getR(ref);
        return rb->calculateK(r, r);
    }
    case SimulationObject::Type::SIMULATION_POINT:
    {
        break;
    }
    }
    return Eigen::Matrix3d::Identity();
}

Matrix3d ImpulseConstraintSolver::calculateL(SimulationObject* so)
{
    switch(so->getType())
    {
    case SimulationObject::Type::FEM_OBJECT:
    {
        return Eigen::Matrix3d::Identity();
    }
    case SimulationObject::Type::RIGID_BODY:
    {
        RigidBody* rb = static_cast<RigidBody*>(so);
        return rb->getInverseInertiaTensorWS();
    }
    case SimulationObject::Type::SIMULATION_POINT:
    {
        break;
    }
    }
    return Eigen::Matrix3d::Identity();
}

Vector ImpulseConstraintSolver::calculateRelativeNormalSpeed(
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

Quaterniond ImpulseConstraintSolver::getOrientation(SimulationObject* so)
{
    if (so->getType() == SimulationObject::Type::RIGID_BODY)
    {
        RigidBody* rb = static_cast<RigidBody*>(so);
        return rb->getOrientation();
    }
    return Eigen::Quaterniond::Identity();
}

Eigen::Vector ImpulseConstraintSolver::getOrientationVelocity(SimulationObject* so)
{
    if (so->getType() == SimulationObject::Type::RIGID_BODY)
    {
        RigidBody* rb = static_cast<RigidBody*>(so);
        return rb->getOrientationVelocity();
    }
    return Eigen::Vector::Zero();
}
