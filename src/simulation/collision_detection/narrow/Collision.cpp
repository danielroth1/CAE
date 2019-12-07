#include "Collision.h"

#include <simulation/SimulationObject.h>

#include <simulation/fem/FEMObject.h>

#include <simulation/rigid/RigidBody.h>



Collision::Collision()
    : mSoA(nullptr)
    , mSoB(nullptr)
{

}

Collision::Collision(
        SimulationObject* soA,
        SimulationObject* soB,
        const Eigen::Vector& pointA,
        const Eigen::Vector& pointB,
        const Eigen::Vector& normal,
        double depth,
        ID vertexIndexA,
        ID vertexIndexB,
        bool isInside)
    : mSoA(soA)
    , mSoB(soB)
    , mPointA(pointA)
    , mPointB(pointB)
    , mNormal(normal)
    , mDepth(depth)
    , mVertexIndexA(vertexIndexA)
    , mVertexIndexB(vertexIndexB)
    , mIsInside(isInside)
{
}

Vector Collision::calculatePositionPreviousA()
{
    return calculatePositionPrevious(mSoA, mPointA, mVertexIndexA);
}

Vector Collision::calculatePositionPreviousB()
{
    return calculatePositionPrevious(mSoB, mPointB, mVertexIndexB);
}

Vector Collision::calculatePositionPrevious(
        SimulationObject* so,
        const Eigen::Vector& point,
        ID vertexIndex)
{
    Eigen::Vector posPrevious;
    switch(so->getType())
    {
    case SimulationObject::FEM_OBJECT:
    {
        posPrevious = static_cast<FEMObject*>(so)->getPositionPrevious(vertexIndex) +
                point - static_cast<FEMObject*>(so)->getPosition(vertexIndex);
        break;
    }
    case SimulationObject::RIGID_BODY:
    {
        RigidBody* rb = static_cast<RigidBody*>(so);
        posPrevious = rb->getPositionPrevious() +
                rb->getOrientationPrevious() * rb->getOrientation().inverse() *
                (point - rb->getPosition());

        // TODO: doesn't work this way because mCollision.getPointA() is
        // in world space coordinates. Should body space coordinates be provided
        // in general? Maybe Use SimulationPointRef in the Colliders?
        break;
    }
    default:
    {
        std::cout << "Warning: Collision::calculatePositionPrevious() is not implemented for this type.\n";
        posPrevious = Eigen::Vector::Zero();
        break;
    }
    }

    return posPrevious;
}


