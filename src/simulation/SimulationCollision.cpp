#include "SimulationCollision.h"
#include "SimulationObject.h"

#include <simulation/rigid/RigidBody.h>

#include <scene/data/references/PolygonBaryRef.h>
#include <scene/data/references/PolygonVectorRef.h>

#include <simulation/fem/FEMObject.h>

SimulationCollision::SimulationCollision()
{

}

SimulationCollision::SimulationCollision(Collision& collision)
    : mCollision(collision)
{
    // Not used yet.
//    mRefA = generateGeometricPointRef(
//                collision.getSimulationObjectA(),
//                collision.getPointA(),
//                collision.getBarycentricCoordiantesA(),
//                collision.getElementIdA());

//    mRefB = generateGeometricPointRef(
//                collision.getSimulationObjectB(),
//                collision.getPointB(),
//                collision.getBarycentricCoordiantesB(),
//                collision.getElementIdB());
}

std::shared_ptr<GeometricPointRef>
SimulationCollision::generateGeometricPointRef(
        SimulationObject* so,
        const Eigen::Vector3d& point,
        const Eigen::Vector4d& bary,
        int elementId) const
{
    // TODO: avoid dynamic memory allocation.

    if (so->getType() == SimulationObject::RIGID_BODY)
    {
        RigidBody* rb = static_cast<RigidBody*>(so);
        Eigen::Vector r = rb->getOrientation().inverse() *
                (point - rb->getPosition());
        return std::make_shared<PolygonVectorRef>(rb->getPolygon().get(), r);
    }
    else if (so->getType() == SimulationObject::FEM_OBJECT)
    {
        FEMObject* femObj = static_cast<FEMObject*>(so);
        std::array<double, 4> baryArray = {bary(0), bary(1), bary(2), bary(3)};
        return std::make_shared<PolygonBaryRef>(
                    femObj->getPolygon().get(), baryArray, elementId);
    }
    return nullptr;
}
