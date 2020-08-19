#include "ImpulseConstraintSolver.h"
#include "SimulationCollision.h"

#include <simulation/collision_detection/narrow/Collision.h>
#include <iostream>
#include <simulation/fem/FEMObject.h>
#include <simulation/rigid/RigidBody.h>
#include <simulation/constraints/BallJoint.h>
#include <simulation/constraints/CollisionConstraint.h>
#include <scene/data/geometric/Polygon3DTopology.h>
#include <scene/data/references/PolygonBaryRef.h>

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
        const std::vector<SimulationCollision>& collisions,
        size_t offset,
        double stepSize,
        double restitution,
        double positionCorrectionFactor,
        double collisionMargin,
        double contactMargin,
        bool positionCorrection)
{
    START_TIMING_SIMULATION("ImpulseConstraintSolver::initializeCollisionConstraints")
    // calculate K, target u rels
    mCollisionConstraints.reserve(mCollisionConstraints.size() + collisions.size());

    for (size_t i = offset; i < collisions.size(); ++i)
    {
        const Collision& c = collisions[i].getCollision();
        mCollisionConstraints.push_back(
                    CollisionConstraint(c, restitution,
                                        positionCorrectionFactor,
                                        collisionMargin,
                                        contactMargin,
                                        positionCorrection,
                                        false));
    }

    for (size_t i = offset; i < mCollisionConstraints.size(); ++i)
    {
        mCollisionConstraints[i].initialize(stepSize);
    }

    STOP_TIMING_SIMULATION
}

void ImpulseConstraintSolver::revalidateCollisionConstraints(
        const std::vector<SimulationCollision>& reusedCollisions,
        double stepSize,
        double restitution,
        double positionCorrectionFactor,
        double collisionMargin,
        double contactMargin,
        bool correctPositionError,
        bool applyWarmStarting)
{
    if (reusedCollisions.empty())
    {
        mCollisionConstraints.clear();
    }
    else
    {
        size_t colIndex = 0;
        size_t count = 0;
        for (size_t i = 0; i < mCollisionConstraints.size(); ++i)
        {
            const CollisionConstraint& cc = mCollisionConstraints[i];

            if (colIndex == reusedCollisions.size())
                break;

            if (cc.getCollision() == reusedCollisions[colIndex].getCollision())
            {
                int reuseCount = cc.getReuseCount() + 1;
                // Uncomment to ignore the first warm starting impulse. The first
                // impulse can be quiet wrong if the objects was just falling
                // down. Often time this just makes the warm starting react too
                // slow, preventing objects comming to a rest, so it is not used.
//                applyWarmStarting = applyWarmStarting && reuseCount > 5;

                // If friction tangents are different, the corresponding
                // friction impulses from previous steps would point in
                // wrong direction. In pracise this doesn't seem to make
                // much of a difference, though.
                bool applyFrictionWarmStarting = false;
//                        reusedConstraints.back().getTangent1().isApprox(cc.getTangent1(), 1e-5) &&
//                        reusedConstraints.back().getTangent2().isApprox(cc.getTangent2(), 1e-5);

                if (applyWarmStarting)
                {
                    new (&mCollisionConstraints[count]) CollisionConstraint(
                                reusedCollisions[colIndex].getCollision(),
                                restitution,
                                positionCorrectionFactor,
                                collisionMargin,
                                contactMargin,
                                correctPositionError,
                                applyWarmStarting,
                                cc.getSumCollisionImpulses(),
                                cc.getSumFrictionImpulses());
                }
                else if (applyWarmStarting && applyFrictionWarmStarting)
                {
                    new (&mCollisionConstraints[count]) CollisionConstraint(
                                reusedCollisions[colIndex].getCollision(),
                                restitution,
                                positionCorrectionFactor,
                                collisionMargin,
                                contactMargin,
                                correctPositionError,
                                applyWarmStarting,
                                cc.getSumCollisionImpulses(),
                                Eigen::Vector2d::Zero());
                }
                else
                {
                    // No warm starting
                    new (&mCollisionConstraints[count]) CollisionConstraint(
                                reusedCollisions[colIndex].getCollision(),
                                restitution,
                                positionCorrectionFactor,
                                collisionMargin,
                                contactMargin,
                                correctPositionError,
                                applyWarmStarting,
                                Eigen::Vector3d::Zero(),
                                Eigen::Vector2d::Zero());
                }

                mCollisionConstraints[count].setReuseCount(reuseCount);
                ++count;
                ++colIndex;
            }
        }

        mCollisionConstraints.resize(count);

        for (size_t i = 0; i < mCollisionConstraints.size(); ++i)
        {
            mCollisionConstraints[i].initialize(stepSize);
        }
    }
}

void ImpulseConstraintSolver::applyWarmStarting()
{
    for (size_t i = 0; i < mCollisionConstraints.size(); ++i)
    {
        mCollisionConstraints[i].applyWarmStarting();
    }
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
        const Eigen::Vector4d& bary,
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

        if (ref.getGeometricType() == GeometricPointRef::Type::GEOMETRIC_VERTEX)
        {
            ID index = ref.getIndex();
            if (index != ILLEGAL_INDEX)
            {
                return 1 / femObj->getMass(index) * Eigen::Matrix3d::Identity();
            }
        }
        else if (ref.getGeometricType() == GeometricPointRef::Type::POLYGON_BARY)
        {
            PolygonBaryRef* polyRef = static_cast<PolygonBaryRef*>(ref.getGeometricPointRef());
            return 1 / femObj->calculateMass(polyRef->getElementId(), polyRef->getBary()) * Eigen::Matrix3d::Identity();
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
        const Eigen::Vector4d& bary,
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
        const Eigen::Vector4d& bary,
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

        double mass =
                bary(0) * femObj->getMass(cell.getVertexIds()[0]) +
                bary(1) * femObj->getMass(cell.getVertexIds()[1]) +
                bary(2) * femObj->getMass(cell.getVertexIds()[2]) +
                bary(3) * femObj->getMass(cell.getVertexIds()[3]);

        for (size_t i = 0; i < 4; ++i)
        {
            ID vertexId = cell.getVertexIds()[i];
            double mass_i = femObj->getMass(vertexId);

            // Paper reference:
            // The finally used impulse can be derived from:
            // "Accurate Collision Response on Polygonal Meshes" by Thalmann.
            // This paper is a bit more abstract and describes general cases
            // of how to deal with force/velocity/position changes on triangle
            // meshes.
            //
            // It is possible to apply different kind of impulses here depending
            // on how barycentric coordinates and masses should be weight.
            // The final impulse should cause a correct valocity change
            // in the contact point:
            // \delta v = \sum^{3}_{i=0} a_i * \delta v_i
            // The following impulses define \delta v_i differently.
            //
            // All impulses are mass independent to improve stability if there
            // is a large mass difference between vertices of a tetrahedron.
            // This is achieved by multiplying mass_i / mass with every
            // impulse. To be more physically accurate, this can be removed.

            // This impulse doesn't work in certain situations but is overall ok.
            // Theoretically, the applied impulse is too weak because the
            // final velocity change multiplies the barycentric coordinates
            // twice, i.e.
            //
            // \delta v_i = a_i * \delta_v
            //
            // with the velocity change in the given point:
            //
            // \delta v = \sum^{3}_{i=0} a_i^2 * \delta v
//            femObj->applyImpulse(cell.getVertexIds()[i], mass_i / mass * bary[i] * impulse);

            // This formulation works but the impulses would then not depend
            // on barycentric coordinates which would make tetrahedrons appear
            // too stiff. The impulse causes a change in velcity for vertex i:
            //
            // \delta v_i = \delta_v
            //
            // The velocit change is:
            //
            // \delta v = \sum^{3}_{i=0} a_i * \delta_v
//            femObj->applyImpulse(cell.getVertexIds()[i], mass_i / mass * impulse);

            // The following impulse can be derived from equation (6) in the
            // referenced paper (see top of comments) ignoring the mass
            // contributions.
            //
            // This impulse seems a bit more sophisticated than the previous
            // because it weighs by barycentric coordinates. This way, an
            // impulse that is applied to a vertex or an edge, e.g. with
            // barycentric coordinates (1, 0, 0, 0) or (0.4, 0.6, 0, 0)
            // only causes a velocity change in those vertices. The previous
            // impulse would cause the same velocity change, even in vertices
            // that are unaffected. It also would treat the first and second
            // vertex of the (0.4, 0.6, 0, 0) example the same.
            //
            // Mathematic formulation:
            // The impulse causes the barycentric coordinates to be weight
            // quadratic, i.e.
            //
            // \delta v_i = a_i / (a_0^2 + a_1^2 + a_2^2 + a_3^2) * \delta v
            //
            // The velocity change at the given point is then:
            //
            // \delta v = \sum^{3}_[i=0} a_i * \delta v_i
            //          = \sum^{3}_[i=0} a_i^2 / (a_0^2 + a_1^2 + a_2^2 + a_3^2) * \delta v
            //
            // which is quadratic and fulfills the condition that the values
            // before \delta v add up to 1.
            femObj->applyImpulse(vertexId, mass_i / mass * bary[i] / bary.squaredNorm() * impulse);
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
