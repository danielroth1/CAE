#ifndef SIMULATIONTHREAD_H
#define SIMULATIONTHREAD_H

#include <multi_threading/StepperThread.h>


class Simulation;
class SimulationControl;

class SimulationThread : public StepperThread
{
public:
    SimulationThread(
            SimulationControl* simulationControl);

    // StepperThread interface
public:
    virtual void initialization() override;
    virtual void beforeStep() override;
    virtual void step() override;

private:
    SimulationControl* mSimulationControl;

};

#endif // SIMULATIONTHREAD_H
