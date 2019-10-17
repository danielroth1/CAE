#include "Simulation.h"
#include "SimulationThread.h"

#include <SimulationControl.h>
#include <memory>
#include <multi_threading/RealTimeStepper.h>
#include <multi_threading/TimeStepper.h>

// TODO: should not obtain simulations with simulation controll getter
// should use observer patten to inform about changes instaed of knowing of
// SimulationControl
SimulationThread::SimulationThread(
        SimulationControl* simulationControl)
    : StepperThread()
    , mSimulationControl(simulationControl)
{
    setTimeStepper(std::make_shared<RealTimeStepper>(0.01));
}

void SimulationThread::setTimeStepSize(double stepSize)
{
    static_cast<RealTimeStepper*>(getTimeStepper())->setTimeStepSize(stepSize);
}

void SimulationThread::initialization()
{
    mSimulationControl->initializeSimulation();
}

void SimulationThread::beforeStep()
{

}

void SimulationThread::step()
{
    mSimulationControl->step();
}
