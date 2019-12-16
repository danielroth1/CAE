#include "ImpulseConstraintSolver.h"

#include <simulation/collision_detection/narrow/Collision.h>
#include <iostream>
#include <simulation/fem/FEMObject.h>
#include <simulation/rigid/RigidBody.h>
#include <simulation/constraints/BallJoint.h>
#include <simulation/constraints/CollisionConstraint.h>
#include <scene/data/geometric/Polygon3DTopology.h>

#include <times/timing.h>

using namespace Eigen;

ImpulseConstraintSolver::ImpulseConstraintSolver()
{

}

ImpulseConstraintSolver::~ImpulseConstraintSolver()
{

}

void ImpulseConstraintSolver::initializeNonCollisionConstraints(double stepSize)
{
    START_TIMING_SIMULATION("ImpulseConstraintSolver::initializeNonCollisionConstraints")
    for (size_t i = 0; i < mConstraints.size(); ++i)
    {
        mConstraints[i]->initialize(stepSize);
    }
    STOP_TIMING_SIMULATION
}

void ImpulseConstraintSolver::initializeCollisionConstraints(
        std::vector<Collision>& collisions,
        double stepSize,
        double restitution,
        double positionCorrectionFactor,
        double collisionMargin)
{
    START_TIMING_SIMULATION("ImpulseConstraintSolver::initializeCollisionConstraints")
    // calculate K, target u rels
    mCollisionConstraints.clear();
    mCollisionConstraints.reserve(collisions.size());

    for (size_t i = 0; i < collisions.size(); ++i)
    {
        Collision& c = collisions[i];
        mCollisionConstraints.push_back(
                    CollisionConstraint(c, restitution,
                                        positionCorrectionFactor,
                                        collisionMargin));
    }

    for (size_t i = 0; i < mCollisionConstraints.size(); ++i)
    {
        mCollisionConstraints[i].initialize(stepSize);
    }

    STOP_TIMING_SIMULATION
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

Matrix3d ImpulseConstraintSolver::calculateK(
        SimulationObject* so,
        const Vector& point,
        const std::array<double, 4>& bary,
        ID elementId)
{
    switch(so->getType())
    {
    case SimulationObject::Type::FEM_OBJECT:
    {
        FEMObject* femObj = static_cast<FEMObject*>(so);
        TopologyCell& cell =
                femObj->getPolygon()->getTopology3D().getCells()[elementId];
        return 1 / (femObj->getMass(cell.getVertexIds()[0]) * bary[0] +
                femObj->getMass(cell.getVertexIds()[1]) * bary[1] +
                femObj->getMass(cell.getVertexIds()[2]) * bary[2] +
                femObj->getMass(cell.getVertexIds()[3]) * bary[3])
                * Eigen::Matrix3d::Identity();
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
        FEMObject* femObj = static_cast<FEMObject*>(ref.getSimulationObject().get());
        ID index = ref.getIndex();
        if (index != ILLEGAL_INDEX)
        {
            return 1 / femObj->getMass(index) * Eigen::Matrix3d::Identity();
        }
        break;
    }
    case SimulationObject::Type::RIGID_BODY:
    {
        RigidBody* rb = static_cast<RigidBody*>(ref.getSimulationObject().get());
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

Vector ImpulseConstraintSolver::calculateSpeed(
        SimulationObject* so,
        const Vector& point,
        const std::array<double, 4>& bary,
        ID elementId)
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
        TopologyCell& cell =
                femObj->getPolygon()->getTopology3D().getCells()[elementId];
        return femObj->getVelocities()[cell.getVertexIds()[0]] * bary[0] +
                femObj->getVelocities()[cell.getVertexIds()[1]] * bary[1] +
                femObj->getVelocities()[cell.getVertexIds()[2]] * bary[2] +
                femObj->getVelocities()[cell.getVertexIds()[3]] * bary[3];
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

void ImpulseConstraintSolver::applyImpulse(
        SimulationObject* so,
        const Vector& impulse,
        const Vector& point,
        const std::array<double, 4>& bary,
        ID elementId)
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
        TopologyCell& cell =
                femObj->getPolygon()->getTopology3D().getCells()[elementId];

        for (size_t i = 0; i < 4; ++i)
        {
            femObj->applyImpulse(cell.getVertexIds()[i], bary[i] * impulse);
        }
        break;
    }
    case SimulationObject::Type::SIMULATION_POINT:
        break;
    }
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

Vector3d ImpulseConstraintSolver::calculateProjectionMatrix(
        const Vector& axis1, const Vector& axis2)
{
    return axis1.cross(axis2);
}

Eigen::Matrix<double, 2, 3>
ImpulseConstraintSolver::calculateProjectionMatrix(const Vector& axis)
{
    Eigen::Vector v(1, 0, 0);
    if (std::fabs(v.dot(axis)) > 0.9999)
        v = Eigen::Vector(0, 1, 0);

    Eigen::Matrix<double, 2, 3> m;
    m.row(0) = axis.cross(v);
    m.row(1) = axis.cross(m.row(0));

    return m;
}
