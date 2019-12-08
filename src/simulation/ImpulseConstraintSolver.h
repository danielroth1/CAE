#ifndef ImpulseConstraintSolver_H
#define ImpulseConstraintSolver_H

#include "ConstraintSolver.h"
#include "SimulationObject.h"

#include <data_structures/DataStructures.h>
#include <memory>
#include <simulation/rigid/RigidBody.h>
#include <vector>

class Collision;
class CollisionConstraint;
class FEMObject;
class SimulationObject;
class SimulationPointRef;

// Becomes invalid after collideAll()
class ImpulseConstraintSolver : public ConstraintSolver
{
public:

    ImpulseConstraintSolver();

    ~ImpulseConstraintSolver() override;

    // Creates and adds collision constraints that resolve the given collisions.
    void initializeNonCollisionConstraints(double stepSize);

    void initializeCollisionConstraints(
            std::vector<Collision>& collisions,
            double stepSize,
            double restitution,
            double cFrictionStatic,
            double cFrictionDynamic,
            double positionCorrectionFactor);

    static Eigen::Vector calculateRelativeNormalSpeed(
            const Eigen::Vector& relativeSpeedA,
            const Eigen::Vector& relativeSpeedB,
            const Eigen::Vector& normal);

    static Eigen::Vector calculateSpeed(
            SimulationObject* so,
            const Eigen::Vector& point,
            const ID vertexIndex);

    static void applyImpulse(
            SimulationObject* so,
            const Eigen::Vector& impulse,
            const Eigen::Vector& point,
            const ID vertexIndex);

    static Eigen::Matrix3d calculateK(
            SimulationObject* so,
            const Eigen::Vector& point,
            const ID vertexIndex);

    static Eigen::Matrix3d calculateK(SimulationPointRef& ref);

    static Eigen::Matrix3d calculateL(SimulationObject* so);

    static Eigen::Vector calculateRelativePoint(
            SimulationObject* so,
            const Eigen::Vector& pointGlobal)
    {
        if (so->getType() == SimulationObject::Type::RIGID_BODY)
        {
            return pointGlobal - static_cast<RigidBody*>(so)->getCenterOfMass();
        }
        return pointGlobal;
    }

    static Eigen::Quaterniond getOrientation(SimulationObject* so);
    static Eigen::Vector getOrientationVelocity(SimulationObject* so);

    static Eigen::Vector3d calculateProjectionMatrix(
            const Eigen::Vector& axis1,
            const Eigen::Vector& axis2);

    static Eigen::Matrix<double, 2, 3>
    calculateProjectionMatrix(const Eigen::Vector& axis);
};

#endif // ImpulseConstraintSolver_H
