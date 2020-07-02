#include "SimulationPointRef.h"

#include <simulation/SimulationObject.h>

#include <scene/data/geometric/Polygon.h>

#include <scene/data/references/PolygonVectorRef.h>
#include <scene/data/references/GeometricVertexRef.h>
#include <scene/data/references/PolygonBaryRef.h>

#include <simulation/fem/FEMObject.h>
#include <simulation/fem/SimulationPoint.h>

#include <simulation/rigid/RigidBody.h>


SimulationPointRef::SimulationPointRef(
        SimulationObject* simObj,
        Polygon* polygon,
        Eigen::Vector r)
    : mGeometricPointRef(std::make_unique<PolygonVectorRef>(polygon, r))
    , mUpdatePolicy(UpdatePolicy::ON_DEMAND)
{
    mSimulationObject = simObj->shared_from_this();
}

SimulationPointRef::SimulationPointRef(
        SimulationObject* simObj,
        ID index)
    : mGeometricPointRef(
          std::make_unique<GeometricVertexRef>(simObj->getGeometricData(), index))
    , mUpdatePolicy(UpdatePolicy::ON_DEMAND)
{
    mSimulationObject = simObj->shared_from_this();
}

SimulationPointRef::SimulationPointRef(
        SimulationObject* simObj,
        Polygon3D* polygon,
        const std::array<double, 4>& bary,
        ID elementId)
    : mGeometricPointRef(
          std::make_unique<PolygonBaryRef>(polygon, bary, elementId))
    , mUpdatePolicy(UpdatePolicy::ON_DEMAND)
{
    mSimulationObject = simObj->shared_from_this();
}

SimulationPointRef::SimulationPointRef(const SimulationPointRef& ref)
    : mGeometricPointRef(
          std::unique_ptr<GeometricPointRef>(ref.mGeometricPointRef->clone()))
    , mSimulationObject(ref.mSimulationObject)
    , mUpdatePolicy(UpdatePolicy::ON_DEMAND)
{

}

SimulationPointRef& SimulationPointRef::operator=(const SimulationPointRef& ref)
{
    mSimulationObject = ref.mSimulationObject;
    mGeometricPointRef =
            std::unique_ptr<GeometricPointRef>(ref.mGeometricPointRef->clone());
    return *this;
}

SimulationPointRef::~SimulationPointRef()
{

}

void SimulationPointRef::applyImpulse(const Vector& impulse)
{
    switch (mSimulationObject->getType())
    {
    case SimulationObject::Type::FEM_OBJECT:
    {
        FEMObject* femObj = static_cast<FEMObject*>(mSimulationObject.get());
        switch (mGeometricPointRef->getType())
        {
        case GeometricPointRef::Type::GEOMETRIC_VERTEX:
        {
            ID index = static_cast<GeometricVertexRef*>(mGeometricPointRef.get())->getIndex();
            femObj->applyImpulse(index, impulse);
            break;
        }
        case GeometricPointRef::Type::POLYGON_BARY:
        {
            PolygonBaryRef* ref = static_cast<PolygonBaryRef*>(mGeometricPointRef.get());
            femObj->applyImpulse(ref->getElementId(), ref->getBary(), impulse);
            break;
        }
        case GeometricPointRef::Type::POLYGON_VECTOR:
        {
            std::cout << "Can not apply impulse for FEMObjects w.r.t. PolygonVectorRef. Unsupported point type.\n";
            break;
        }
        }
        break;
    }
    case SimulationObject::Type::RIGID_BODY:
    {
        RigidBody* rb = static_cast<RigidBody*>(mSimulationObject.get());
        switch (mGeometricPointRef->getType())
        {
        case GeometricPointRef::Type::GEOMETRIC_VERTEX:
        {
            std::cout << "Can not apply impulse for RigidBody w.r.t. GeometricVertexRef. Unsupported point type.\n";
            break;
        }
        case GeometricPointRef::Type::POLYGON_BARY:
        {
            std::cout << "Can not apply impulse for RigidBody w.r.t. PolygonBaryRef. Unsupported point type.\n";
            break;
        }
        case GeometricPointRef::Type::POLYGON_VECTOR:
        {
            rb->applyImpulse(mPoint - rb->getCenterOfMass(), impulse);
        }
        }
        break;
    }
    case SimulationObject::Type::SIMULATION_POINT:
    {
        // Do nothing
//        std::cout << "Can not apply impulse w.r.t. SimulationPoint.\n";
        break;
    }
    }
}

Vector SimulationPointRef::calculateSpeed()
{
    switch (mSimulationObject->getType())
    {
    case SimulationObject::Type::FEM_OBJECT:
    {
        FEMObject* femObj = static_cast<FEMObject*>(mSimulationObject.get());
        switch (mGeometricPointRef->getType())
        {
        case GeometricPointRef::Type::GEOMETRIC_VERTEX:
        {
            ID index = static_cast<GeometricVertexRef*>(mGeometricPointRef.get())->getIndex();
            return femObj->getVelocities()[index];
        }
        case GeometricPointRef::Type::POLYGON_BARY:
        {
            PolygonBaryRef* ref = static_cast<PolygonBaryRef*>(mGeometricPointRef.get());
            return femObj->calculateVelocity(ref->getElementId(), ref->getBary());
        }
        case GeometricPointRef::Type::POLYGON_VECTOR:
        {
            break;
        }
        }
        break;
    }
    case SimulationObject::Type::RIGID_BODY:
    {
        RigidBody* rb = static_cast<RigidBody*>(mSimulationObject.get());
        switch (mGeometricPointRef->getType())
        {
        case GeometricPointRef::Type::GEOMETRIC_VERTEX:
        {
            break;
        }
        case GeometricPointRef::Type::POLYGON_BARY:
        {
            break;
        }
        case GeometricPointRef::Type::POLYGON_VECTOR:
        {
            return rb->calculateSpeedAt(mPoint - rb->getCenterOfMass());
        }
        }
        break;
    }
    case SimulationObject::Type::SIMULATION_POINT:
    {
        break;
    }
    }
    return Eigen::Vector::Zero();
}

void SimulationPointRef::update()
{
    switch (mUpdatePolicy)
    {
    case UpdatePolicy::ON_DEMAND:
        // Nothing to do here because points are calculated on demand directly
        // in the getter.
        break;
    case UpdatePolicy::ON_UPDATE_CALL:
        mPoint = calculatePoint();
        break;
    }
}

void SimulationPointRef::updatePrevious()
{
    switch (mUpdatePolicy)
    {
    case UpdatePolicy::ON_DEMAND:
        // Nothing to do here because points are calculated on demand directly
        // in the getter.
        break;
    case UpdatePolicy::ON_UPDATE_CALL:
        mPointPrevious = calculatePointPrevious();
        break;
    }
}

void SimulationPointRef::setUpdatePolicy(SimulationPointRef::UpdatePolicy policy)
{
    mUpdatePolicy = policy;
}

const std::shared_ptr<SimulationObject>& SimulationPointRef::getSimulationObject() const
{
    return mSimulationObject;
}

GeometricPointRef* SimulationPointRef::getGeometricPointRef() const
{
    return mGeometricPointRef.get();
}

GeometricData* SimulationPointRef::getGeometricData() const
{
    return mGeometricPointRef->getGeometricData();
}

Eigen::Vector SimulationPointRef::getPoint()
{
    switch (mUpdatePolicy)
    {
    case UpdatePolicy::ON_DEMAND:
        return calculatePoint();
    case UpdatePolicy::ON_UPDATE_CALL:
        return mPoint;
    }

    std::cout << "Added additional update policy but didn't handle it.\n";
    return calculatePoint();
}

Vector SimulationPointRef::getPointPrevious()
{
    switch (mUpdatePolicy)
    {
    case UpdatePolicy::ON_DEMAND:
        return calculatePointPrevious();
    case UpdatePolicy::ON_UPDATE_CALL:
        return mPointPrevious;
    }

    std::cout << "Added additional update policy but didn't handle it.\n";
    return calculatePoint();
}

GeometricPointRef::Type SimulationPointRef::getGeometricType() const
{
    return mGeometricPointRef->getType();
}

ID SimulationPointRef::getIndex() const
{
    return mGeometricPointRef->getIndex();
}

void SimulationPointRef::accept(GeometricPointRefVisitor& visitor)
{
    mGeometricPointRef->accept(visitor);
}

Vector SimulationPointRef::getGeometricPoint() const
{
    return mGeometricPointRef->getPoint();
}

Vector SimulationPointRef::getPoint(SimulationPoint& sp)
{
    return sp.getPosition(0);
}

Vector SimulationPointRef::getPoint(RigidBody& rb)
{
    return rb.getPosition() + rb.getR(*this);
}

Vector SimulationPointRef::getPoint(FEMObject& femObj)
{
    if (mGeometricPointRef->getType() == GeometricPointRef::Type::GEOMETRIC_VERTEX)
    {
        ID index = static_cast<GeometricVertexRef*>(mGeometricPointRef.get())->getIndex();
        return femObj.getPosition(index);
    }
    else if (mGeometricPointRef->getType() == GeometricPointRef::Type::POLYGON_BARY)
    {
        return static_cast<PolygonBaryRef*>(mGeometricPointRef.get())->getPoint();
    }
    return Eigen::Vector::Zero();
}

Vector SimulationPointRef::getPreviousPoint(SimulationPoint& sp)
{
    return sp.getPosition(0);
}

Vector SimulationPointRef::getPreviousPoint(RigidBody& rb)
{
    return rb.getPositionPrevious() +
            rb.getOrientationPrevious().toRotationMatrix() * rb.getR(*this);
}

Vector SimulationPointRef::getPreviousPoint(FEMObject& femObj)
{
    if (mGeometricPointRef->getType() == GeometricPointRef::Type::GEOMETRIC_VERTEX)
    {
        ID index = static_cast<GeometricVertexRef*>(mGeometricPointRef.get())->getIndex();
        return femObj.getPositionPrevious(index);
    }
    return Eigen::Vector::Zero();
}

Vector SimulationPointRef::calculatePoint()
{
    switch(mSimulationObject->getType())
    {
    case SimulationObject::Type::FEM_OBJECT:
        return getPoint(*static_cast<FEMObject*>(mSimulationObject.get()));
    case SimulationObject::Type::RIGID_BODY:
        return getPoint(*static_cast<RigidBody*>(mSimulationObject.get()));
    case SimulationObject::Type::SIMULATION_POINT:
        return getPoint(*static_cast<SimulationPoint*>(mSimulationObject.get()));
    }
}

Vector SimulationPointRef::calculatePointPrevious()
{
    switch(mSimulationObject->getType())
    {
    case SimulationObject::Type::FEM_OBJECT:
        return getPreviousPoint(*static_cast<FEMObject*>(mSimulationObject.get()));
    case SimulationObject::Type::RIGID_BODY:
        return getPreviousPoint(*static_cast<RigidBody*>(mSimulationObject.get()));
    case SimulationObject::Type::SIMULATION_POINT:
        return getPreviousPoint(*static_cast<SimulationPoint*>(mSimulationObject.get()));
    }
}
