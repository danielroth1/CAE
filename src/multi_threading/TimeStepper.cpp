#include "TimeStepper.h"

TimeStepper::TimeStepper(int sleepTime)
    : mSleepTime(sleepTime)
{

}

void TimeStepper::reset()
{

}

void TimeStepper::startStep()
{

}

int TimeStepper::finishStep()
{
    return mSleepTime; // ms
}
