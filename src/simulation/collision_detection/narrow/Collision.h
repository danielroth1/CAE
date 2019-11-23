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
            ID vertexIndexB,
            bool isInside);

    void revert()
    {
        Eigen::Vector temp;
        temp = mPointA;
        mPointA = mPointB;
        mPointB = temp;

        SimulationObject* tempSo = mSoA;
        mSoA = mSoB;
        mSoB = tempSo;

        ID tempVertexIndex = mVertexIndexA;
        mVertexIndexA = mVertexIndexB;
        mVertexIndexB = tempVertexIndex;

        mNormal = -mNormal;
    }

    void setNormal(const Eigen::Vector& normal)
    {
        mNormal = normal;
    }

    SimulationObject* getSimulationObjectA()
    {
        return mSoA;
    }
    SimulationObject* getSimulationObjectB()
    {
        return mSoB;
    }
    const Eigen::Vector& getPointA() const
    {
        return mPointA;
    }
    const Eigen::Vector& getPointB() const
    {
        return mPointB;
    }
    const Eigen::Vector& getNormal() const
    {
        return mNormal;
    }
    double getDepth() const
    {
        return mDepth;
    }
    ID getVertexIndexA() const
    {
        return mVertexIndexA;
    }
    ID getVertexIndexB() const
    {
        return mVertexIndexB;
    }
    bool isInside() const
    {
        return mIsInside;
    }

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
    bool mIsInside;
};

#endif // COLLISION_H
