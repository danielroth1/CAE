#ifndef REALTIMESTEPPER_H
#define REALTIMESTEPPER_H

#include "TimeStepper.h"

#include <chrono>
#include <ctime>

typedef std::chrono::time_point<std::chrono::system_clock> TimePoint;
typedef std::chrono::duration<double> Duration; // duration in seconds
typedef std::time_t Time;

// Sleep time between time steps is chosen so that exactly enough time steps
// are executed to fill out a second. For smaller step sizes, more simulation
// step calles are executed if possible.
// If the time spent per simulation step exceeds the time step size, the sleep
// time is simply zero. In that case the simulation will not run in real time.
//
class RealTimeStepper : public TimeStepper
{
public:
    // \param timeStepSize - the time step size in seconds.
    RealTimeStepper(double timeStepSize);

    // Sets the time step size in seconds.
    void setTimeStepSize(double timeStepSize);

    // TimeStepper interface
public:
    virtual void reset();
    virtual void startStep();
    virtual int finishStep();

private:

    double mMaxMSPerTimeStep;

    TimePoint mPrevTime;

    int mMaxSteps;

};

#endif // REALTIMESTEPPER_H
