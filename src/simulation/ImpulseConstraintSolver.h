#ifndef ImpulseConstraintSolver_H
#define ImpulseConstraintSolver_H

#include "ConstraintSolver.h"

#include <data_structures/DataStructures.h>
#include <memory>
#include <vector>

class Collision;
class CollisionConstraint;
class FEMObject;
class RigidBody;
class SimulationObject;

// Becomes invalid after collideAll()
class ImpulseConstraintSolver : public ConstraintSolver
{
public:

    ImpulseConstraintSolver();

    ~ImpulseConstraintSolver() override;

    void initialize(std::vector<Collision>& collisions,
                    double stepSize,
                    double restitution,
                    double maxCollisionDistance);

    void solveConstraints(int maxIterations, double maxConstraintError) override;

    // Returns true if the constraint was already valid and no impulse was
    // applied, else returns false.
    bool solveConstraint(CollisionConstraint& cc, double maxConstraintError) override;

    bool solveConstraint(BallJoint& ballJoint, double maxConstraintError) override;

private:

    Eigen::Matrix3d calculateK(
            SimulationObject* so,
            const Eigen::Vector& point,
            const ID vertexIndex);

    Eigen::Vector calculateRelativeSpeed(
            const Eigen::Vector& relativeSpeedA,
            const Eigen::Vector& relativeSpeedB,
            const Eigen::Vector& normal);

    Eigen::Vector calculateSpeed(
            SimulationObject* so,
            const Eigen::Vector& point,
            const ID vertexIndex);

    void applyImpulse(
            SimulationObject* so,
            const Eigen::Vector& impulse,
            const Eigen::Vector& point,
            const ID vertexIndex);

    Eigen::Vector calculateRelativePoint(
            SimulationObject* so,
            const Eigen::Vector& pointGlobal);

};

#endif // ImpulseConstraintSolver_H
