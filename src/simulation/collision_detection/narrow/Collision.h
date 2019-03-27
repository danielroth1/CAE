#ifndef COLLISION_H
#define COLLISION_H

#include <data_structures/DataStructures.h>
#include <simulation/references/SimulationPointRef.h>

class Collision
{
public:
    Collision();

    Collision(
            SimulationObject* soA,
            SimulationObject* soB,
            const Eigen::Vector& pointA,
            const Eigen::Vector& pointB,
            const Eigen::Vector& normal,
            double depth,
            ID vertexIndexA,
            ID vertexIndexB);

    SimulationObject* getSimulationObjectA();
    SimulationObject* getSimulationObjectB();
    const Eigen::Vector& getPointA() const;
    const Eigen::Vector& getPointB() const;
    const Eigen::Vector& getNormal() const;
    double getDepth() const;
    ID getVertexIndexA() const;
    ID getVertexIndexB() const;

    // TODO: should this class know of CollisionSpheres?
    // it would be able to caluclate the current distance/ current PointA and PointB (before the collision)

private:
    SimulationObject* mSoA;
    SimulationObject* mSoB;
    Eigen::Vector mPointA;
    Eigen::Vector mPointB;
    Eigen::Vector mNormal;
    double mDepth; // penetration depth
    ID mVertexIndexA; // For deformable objects.
    ID mVertexIndexB; // For deformable objects.
};

#endif // COLLISION_H
