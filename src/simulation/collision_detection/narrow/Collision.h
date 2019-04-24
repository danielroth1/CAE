#ifndef COLLISION_H
#define COLLISION_H

#include <data_structures/DataStructures.h>
#include <simulation/references/SimulationPointRef.h>

class Collision
{
public:
    Collision();

    Collision(
            const std::shared_ptr<SimulationObject>& soA,
            const std::shared_ptr<SimulationObject>& soB,
            const Eigen::Vector& pointA,
            const Eigen::Vector& pointB,
            const Eigen::Vector& normal,
            double depth,
            ID vertexIndexA,
            ID vertexIndexB);

    const std::shared_ptr<SimulationObject>& getSimulationObjectA();
    const std::shared_ptr<SimulationObject>& getSimulationObjectB();
    const Eigen::Vector& getPointA() const;
    const Eigen::Vector& getPointB() const;
    const Eigen::Vector& getNormal() const;
    double getDepth() const;
    ID getVertexIndexA() const;
    ID getVertexIndexB() const;

    // TODO: should this class know of CollisionSpheres?
    // it would be able to caluclate the current distance/ current PointA and PointB (before the collision)

private:
    // This ugly construct is used for performance reasons. We don't want to
    // reference two shared pointers each time a Collision occurs (which could
    // happen thousands of times per time step). Instead we only store a pointer
    // to the shared ptr. The usual reference (i.e.
    // const std::shared_ptr<SimulationObject>&) isn't used here, because we
    // want to beeing able to initialized mSoA and mSoB with nullptr.
    const std::shared_ptr<SimulationObject>* mSoA;
    const std::shared_ptr<SimulationObject>* mSoB;
    Eigen::Vector mPointA;
    Eigen::Vector mPointB;
    Eigen::Vector mNormal;
    double mDepth; // penetration depth
    ID mVertexIndexA; // For deformable objects.
    ID mVertexIndexB; // For deformable objects.
};

#endif // COLLISION_H
