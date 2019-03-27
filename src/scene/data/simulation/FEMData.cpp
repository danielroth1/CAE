#include "FEMData.h"

#include <scene/data/SimulationData.h>
#include <scene/data/SimulationDataVisitor.h>
#include <simulation/fem/FEMObject.h>


FEMData::FEMData(
        FEMObject* femObject,
        Vectors& initialPositions,
        Vectors& positions,
        Vectors& velocities)
    : mFemObject(femObject)
    , mInitialPositions(initialPositions)
    , mPositions(positions)
    , mVelocities(velocities)
{

}

FEMObject* FEMData::getFEMObject()
{
    return mFemObject;
}

Vectors& FEMData::getInitialPositions()
{
    return mInitialPositions;
}

Vectors& FEMData::getPositions()
{
    return mPositions;
}

Vectors& FEMData::getVelocities()
{
    return mVelocities;
}

std::vector<ElasticMaterial>& FEMData::getElasticMaterialProperties()
{
    return mElasticMaterialProperties;
}

void FEMData::visit(SimulationDataVisitor& visitor)
{
    visitor.visit(*this);
}

SimulationObject* FEMData::getSimulationObject()
{
    return mFemObject;
}
