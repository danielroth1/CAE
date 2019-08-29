#ifndef SIMULATIONCONTROLLISTENER_H
#define SIMULATIONCONTROLLISTENER_H

#include <memory>

class Constraint;
class SimulationObject;

class SimulationControlListener
{
public:
    SimulationControlListener();
    virtual ~SimulationControlListener();

    virtual void onSimulationObjectAdded(SimulationObject* so) = 0;
    virtual void onSimulationObjectRemoved(SimulationObject* so) = 0;

    virtual void onConstraintAdded(const std::shared_ptr<Constraint>& c) = 0;
    virtual void onConstraintRemoved(const std::shared_ptr<Constraint>& c) = 0;
};

#endif // SIMULATIONCONTROLLISTENER_H
