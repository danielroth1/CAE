#include "SimulationObject.h"
#include "modules/mesh_converter/MeshConverter.h"
#include <QDebug>
#include <iostream>

Domain*SimulationObject::getDomain()
{
    return mDomain;
}

SimulationObject::SimulationObject(Domain* domain)
    : mDomain(domain)
{

}

SimulationObject::~SimulationObject()
{

}
