#ifndef SIMULATIONPOINTREFVECTOR_H
#define SIMULATIONPOINTREFVECTOR_H

#include <data_structures/DataStructures.h>

#include <data_structures/references/PointRefCollectionVector.h>

#include <simulation/SimulationObject.h>

class SimulationPointRefVector : public PointRefCollectionVector<SimulationObject*, Eigen::Vector&>
{
public:
    SimulationPointRefVector(SimulationObject* object);

    // PointRefCollectionVector interface
public:
    virtual Eigen::Vector& getPoint(ID index);
};

#endif // SIMULATIONPOINTREFVECTOR_H
