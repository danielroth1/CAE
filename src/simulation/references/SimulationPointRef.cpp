#include "SimulationPointRef.h"

#include <simulation/SimulationObject.h>

#include <scene/data/geometric/Polygon.h>

#include <scene/data/references/PolygonVectorRef.h>
#include <scene/data/references/GeometricVertexRef.h>

#include <simulation/fem/FEMObject.h>
#include <simulation/fem/SimulationPoint.h>

#include <simulation/rigid/RigidBody.h>


SimulationPointRef::SimulationPointRef(
        SimulationObject* simObj,
        Polygon* polygon,
        Eigen::Vector r)
    : mGeometricPointRef(std::make_unique<PolygonVectorRef>(polygon, r))
    , mGetSimulationPointDispatcher(*this)
    , mGetPrevSimulationPointDispatcher(*this)
{
    mSimulationObject = simObj->shared_from_this();
}

SimulationPointRef::SimulationPointRef(
        SimulationObject* simObj,
        ID index)
    : mGeometricPointRef(
          std::make_unique<GeometricVertexRef>(simObj->getGeometricData(), index))
    , mGetSimulationPointDispatcher(*this)
    , mGetPrevSimulationPointDispatcher(*this)
{
    mSimulationObject = simObj->shared_from_this();
}

SimulationPointRef::SimulationPointRef(const SimulationPointRef& ref)
    : mGeometricPointRef(
          std::unique_ptr<GeometricPointRef>(ref.mGeometricPointRef->clone()))
    , mSimulationObject(ref.mSimulationObject)
    , mGetSimulationPointDispatcher(*this)
    , mGetPrevSimulationPointDispatcher(*this)
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

const std::shared_ptr<SimulationObject>& SimulationPointRef::getSimulationObject() const
{
    return mSimulationObject;
}

GeometricPointRef* SimulationPointRef::getGeometricPointRef() const
{
    return mGeometricPointRef.get();
}

Eigen::Vector SimulationPointRef::getPoint()
{
    mSimulationObject->accept(mGetSimulationPointDispatcher);
    return mGetSimulationPointDispatcher.point;
}

Vector SimulationPointRef::getPointPrevious()
{
    mSimulationObject->accept(mGetPrevSimulationPointDispatcher);
    return mGetPrevSimulationPointDispatcher.point;
}

ID SimulationPointRef::getIndex() const
{
    return mGeometricPointRef->getIndex();
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

SimulationPointRef::GetSimulationPointDispatcher::GetSimulationPointDispatcher(
        SimulationPointRef& _ref)
    : ref(_ref)
{
}

SimulationPointRef::GetPrevSimulationPointDispatcher::GetPrevSimulationPointDispatcher(
        SimulationPointRef& _ref)
    : ref(_ref)
{

}
