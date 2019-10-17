#include "TimeStepper.h"

TimeStepper::TimeStepper(int sleepTime)
    : mSleepTime(sleepTime)
{

}

TimeStepper::~TimeStepper()
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
