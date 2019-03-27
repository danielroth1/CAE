#ifndef FEMDATA_H
#define FEMDATA_H

#include "data_structures/DataStructures.h"

#include <scene/data/SimulationData.h>
#include <simulation/ElasticMaterial.h>

class FEMObject;
class SimulationDataVisitor;

// Stores the data that needs to be serialized when saving a FEMObject.
// TODO: I don't think it needs all those references.
// much data that needs to be stores is not here, e.g. masses for each finite element
class FEMData : public SimulationData
{
public:
    FEMData(FEMObject* femObject,
            Vectors& initialPositions,
            Vectors& positions,
            Vectors& velocities);

    FEMObject* getFEMObject();
    Vectors& getInitialPositions();
    Vectors& getPositions();
    Vectors& getVelocities();

    std::vector<ElasticMaterial>& getElasticMaterialProperties();

    // SimulationData interface
public:
    virtual void visit(SimulationDataVisitor& visitor);
    virtual SimulationObject* getSimulationObject();

private:

    FEMObject* mFemObject;

    // particles
    Vectors mInitialPositions;
    Vectors mPositions;
    Vectors mVelocities;

    // FEM

    std::vector<ElasticMaterial> mElasticMaterialProperties;


};

#endif // FEMDATA_H
