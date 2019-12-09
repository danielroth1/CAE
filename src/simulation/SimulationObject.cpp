#include "SimulationObject.h"
#include "modules/mesh_converter/MeshConverter.h"
#include <QDebug>
#include <iostream>

void SimulationObject::setFrictionDynamic(double frictionDynamic)
{
    mFrictionDynamic = frictionDynamic;
}

double SimulationObject::getFrictionDynamic() const
{
    return mFrictionDynamic;
}

void SimulationObject::setFrictionStatic(double frictionStatic)
{
    mFrictionStatic = frictionStatic;
}

double SimulationObject::getFrictionStatic() const
{
    return mFrictionStatic;
}

Domain*SimulationObject::getDomain()
{
    return mDomain;
}

SimulationObject::SimulationObject(Domain* domain, Type type)
    : mDomain(domain)
    , mType(type)
{
    mFrictionStatic = 0.05;
    mFrictionDynamic = 0.8;
}

SimulationObject::~SimulationObject()
{

}
