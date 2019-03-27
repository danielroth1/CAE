#include "SimulationPointRef.h"

#include <simulation/SimulationObject.h>

#include <scene/data/geometric/Polygon.h>

#include <scene/data/references/PolygonVectorRef.h>
#include <scene/data/references/GeometricVertexRef.h>


SimulationPointRef::SimulationPointRef(
        SimulationObject* simObj,
        Polygon* polygon,
        Eigen::Vector r)
    : mGeometricPointRef(std::make_unique<PolygonVectorRef>(polygon, r))
    , mSimulationObject(simObj)
{

}

SimulationPointRef::SimulationPointRef(
        SimulationObject* simOb,
        ID index)
    : mGeometricPointRef(std::make_unique<GeometricVertexRef>(simOb->getGeometricData(), index))
    , mSimulationObject(simOb)
{

}

SimulationPointRef::SimulationPointRef(const SimulationPointRef& ref)
    : mGeometricPointRef(std::unique_ptr<GeometricPointRef>(ref.mGeometricPointRef->clone()))
    , mSimulationObject(ref.mSimulationObject)
{

}

SimulationPointRef& SimulationPointRef::operator=(const SimulationPointRef& ref)
{
    mSimulationObject = ref.mSimulationObject;
    mGeometricPointRef = std::unique_ptr<GeometricPointRef>(ref.mGeometricPointRef->clone());
    return *this;
}

SimulationPointRef::~SimulationPointRef()
{

}

SimulationObject* SimulationPointRef::getSimulationObject() const
{
    return mSimulationObject;
}

GeometricPointRef* SimulationPointRef::getGeometricPointRef() const
{
    return mGeometricPointRef.get();
}

Eigen::Vector SimulationPointRef::getPoint() const
{
    return mGeometricPointRef->getPoint();
}
