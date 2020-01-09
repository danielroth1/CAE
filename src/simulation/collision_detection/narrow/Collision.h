#ifndef COLLISION_H
#define COLLISION_H

#include <data_structures/DataStructures.h>
#include <simulation/references/SimulationPointRef.h>

// Stores for
// -> Rigids:
//  The collision point in the global coorindate system.
// -> FEM Object:
//  The element id and barycentric corrdinates of the collision point.
//
// The collision normal points from the collision point of object A to
// collision point of object B. Because the objects are intersecting, the normal
// points from B to A.
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

    Eigen::Vector calculatePositionPreviousA();
    Eigen::Vector calculatePositionPreviousB();

    static Eigen::Vector calculatePositionPrevious(
            SimulationObject* so,
            const Eigen::Vector& point,
            const Eigen::Vector4d& bary,
            ID elementId);

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

    Eigen::Vector4d& getBarycentricCoordiantesA()
    {
        return mBaryA;
    }

    Eigen::Vector4d& getBarycentricCoordiantesB()
    {
        return mBaryB;
    }

    void setElementIdA(ID elementId)
    {
        mElementIdA = elementId;
    }

    ID getElementIdA() const
    {
        return mElementIdA;
    }

    void setElementIdB(ID elementId)
    {
        mElementIdB = elementId;
    }

    ID getElementIdB() const
    {
        return mElementIdB;
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

    // Barycentric coordinates of the affected element / triangle.
    // If its a triangle, only the 3 numbers are non-zero.
    // If its an edge, only 2 numbers are non-zero.
    Eigen::Vector4d mBaryA;
    Eigen::Vector4d mBaryB;
    ID mElementIdA;
    ID mElementIdB;
};

#endif // COLLISION_H
