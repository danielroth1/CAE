#ifndef SIMULATIONDATA_H
#define SIMULATIONDATA_H

// Included
#include <scene/scene_graph/SceneData.h>

class SimulationDataVisitor;
class SimulationObject;

class SimulationData
{
public:

    virtual ~SimulationData();

    virtual void visit(SimulationDataVisitor& visitor) = 0;

    virtual SimulationObject* getSimulationObject() = 0;
};

#endif // SIMULATIONDATA_H
