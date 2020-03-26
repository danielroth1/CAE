#include "Collision.h"

#include <simulation/SimulationObject.h>

#include <simulation/fem/FEMObject.h>

#include <simulation/rigid/RigidBody.h>

#include <scene/data/geometric/Polygon3DTopology.h>
#include <scene/data/geometric/TopologyCell.h>



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

Vector Collision::calculatePositionPreviousA() const
{
    return calculatePositionPrevious(mSoA, mPointA, mBaryA, mElementIdA);
}

Vector Collision::calculatePositionPreviousB() const
{
    return calculatePositionPrevious(mSoB, mPointB, mBaryB, mElementIdB);
}

Vector Collision::calculatePositionPrevious(
        SimulationObject* so,
        const Eigen::Vector& point,
        const Eigen::Vector4d& bary,
        ID elementId)
{
    Eigen::Vector posPrevious;
    switch(so->getType())
    {
    case SimulationObject::FEM_OBJECT:
    {
        FEMObject* femObj = static_cast<FEMObject*>(so);
        TopologyCell& cell =
                femObj->getPolygon()->getTopology3D().getCells()[elementId];

        posPrevious =
                (femObj->getPositionPrevious(cell.getVertexIds()[0]) * bary[0] +
                femObj->getPositionPrevious(cell.getVertexIds()[1]) * bary[1] +
                femObj->getPositionPrevious(cell.getVertexIds()[2]) * bary[2] +
                femObj->getPositionPrevious(cell.getVertexIds()[3]) * bary[3]);
//        posPrevious =
//                (femObj->getPositionPrevious(cell.getVertexIds()[0]) * bary[0] +
//                femObj->getPositionPrevious(cell.getVertexIds()[1]) * bary[1] +
//                femObj->getPositionPrevious(cell.getVertexIds()[2]) * bary[2] +
//                femObj->getPositionPrevious(cell.getVertexIds()[3]) * bary[3]) +
//                point -
//                (femObj->getPosition(cell.getVertexIds()[0]) * bary[0] +
//                femObj->getPosition(cell.getVertexIds()[1]) * bary[1] +
//                femObj->getPosition(cell.getVertexIds()[2]) * bary[2] +
//                femObj->getPosition(cell.getVertexIds()[3]) * bary[3]);
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


