#ifndef SIMULATIONCONTROLLISTENER_H
#define SIMULATIONCONTROLLISTENER_H

class SimulationObject;

class SimulationControlListener
{
public:
    SimulationControlListener();
    virtual ~SimulationControlListener();

    virtual void onSimulationObjectAdded(SimulationObject* so) = 0;
    virtual void onSimulationObjectRemoved(SimulationObject* so) = 0;

    virtual void onConstraintAdded(SimulationObject* so) = 0;
    virtual void onConstraintRemoved(SimulationObject* so) = 0;
};

#endif // SIMULATIONCONTROLLISTENER_H
