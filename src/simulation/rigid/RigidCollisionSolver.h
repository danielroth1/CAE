#ifndef RIGIDCOLLISIONSOLVER_H
#define RIGIDCOLLISIONSOLVER_H

#include <data_structures/DataStructures.h>
#include <memory>
#include <vector>

class Collision;
class FEMObject;
class RigidBody;
class SimulationObject;

// Becomes invalid after collideAll()
class RigidCollisionSolver
{
    // Forward declaration of nested class must be inside class definition.
    class CollisionConstraint;

public:

    RigidCollisionSolver();

    ~RigidCollisionSolver();

    void initialize(std::vector<Collision>& collisions,
                    double stepSize,
                    double restitution,
                    double maxCollisionDistance);

    void solveConstraints(int maxIterations, double maxConstraintError);

    // Returns true if the constraint was already valid and no impulse was
    // applied, else returns false.
    bool solveConstraint(CollisionConstraint& cc, double maxConstraintError);


private:
    class CollisionConstraint
    {
    public:
        CollisionConstraint(Collision& _collision,
                            Eigen::Vector _targetUNormalRel,
                            Eigen::Vector _sumOfAllAppliedImpulses,
                            double _impulseFactor);

        Collision& collision;
        Eigen::Vector targetUNormalRel;
        Eigen::Vector sumOfAllAppliedImpulses;
        double impulseFactor; // 1 / (n^T K_aa + K_bb + n) * n
        bool impulseApplied;
    };

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

    std::vector<CollisionConstraint> mCollisionConstraints;
};

#endif // RIGIDCOLLISIONSOLVER_H
