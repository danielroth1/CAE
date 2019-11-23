#include "Collision.h"



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
