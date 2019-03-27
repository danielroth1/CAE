#ifndef SIMULATIONPOINTREFMAP_H
#define SIMULATIONPOINTREFMAP_H

#include <data_structures/DataStructures.h>

#include <data_structures/references/PointRefCollectionMap.h>

#include <simulation/SimulationObject.h>

class SimulationPointRefMap : public PointRefCollectionMap<SimulationObject*, Eigen::Vector&>
{
public:
    SimulationPointRefMap();

    // PointRefCollectionMap interface
public:
    virtual Eigen::Vector& getPoint(SimulationObject* object, ID index);
};

#endif // SIMULATIONPOINTREFMAP_H
