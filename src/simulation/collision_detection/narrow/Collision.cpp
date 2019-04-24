#include "Collision.h"



Collision::Collision()
    : mSoA(nullptr)
    , mSoB(nullptr)
{

}

Collision::Collision(
        const std::shared_ptr<SimulationObject>& soA,
        const std::shared_ptr<SimulationObject>& soB,
        const Eigen::Vector& pointA,
        const Eigen::Vector& pointB,
        const Eigen::Vector& normal,
        double depth,
        ID vertexIndexA,
        ID vertexIndexB)
    : mSoA(&soA)
    , mSoB(&soB)
    , mPointA(pointA)
    , mPointB(pointB)
    , mNormal(normal)
    , mDepth(depth)
    , mVertexIndexA(vertexIndexA)
    , mVertexIndexB(vertexIndexB)
{
}

const std::shared_ptr<SimulationObject>& Collision::getSimulationObjectA()
{
    return *mSoA;
}

const std::shared_ptr<SimulationObject>& Collision::getSimulationObjectB()
{
    return *mSoB;
}

const Eigen::Vector& Collision::getPointA() const
{
    return mPointA;
}

const Eigen::Vector& Collision::getPointB() const
{
    return mPointB;
}

const Eigen::Vector& Collision::getNormal() const
{
    return mNormal;
}

double Collision::getDepth() const
{
    return mDepth;
}

ID Collision::getVertexIndexA() const
{
    return mVertexIndexA;
}

ID Collision::getVertexIndexB() const
{
    return mVertexIndexB;
}
