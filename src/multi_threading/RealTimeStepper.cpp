#include "RealTimeStepper.h"

#include <cmath>
#include <iostream>


RealTimeStepper::RealTimeStepper(double timeStepSize)
    : TimeStepper(1e+4) // 10 ms but isn't used anyways
{
    setTimeStepSize(timeStepSize);
}

void RealTimeStepper::setTimeStepSize(double timeStepSize)
{
    mMaxMSPerTimeStep = timeStepSize * 1e+6;
}

void RealTimeStepper::reset()
{

}

void RealTimeStepper::startStep()
{
    mPrevTime = std::chrono::system_clock::now();
}

int RealTimeStepper::finishStep()
{
    TimePoint current = std::chrono::system_clock::now();
    Duration passedTime = current - mPrevTime;
    double passedTimeMS = passedTime.count() * 1e+6;

    double remainingTime = mMaxMSPerTimeStep - passedTimeMS;

//    std::cout << mMaxMSPerTimeStep << " - " << passedTimeMS << " = " << remainingTime << "\n";
    return std::max(0, static_cast<int>(std::floor(remainingTime)));
}
