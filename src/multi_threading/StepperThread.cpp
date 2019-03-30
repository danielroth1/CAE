#include "Domain.h"
#include "StepperThread.h"
#include "TimeStepper.h"

#include <iostream>
#include <chrono>

StepperThread::StepperThread()
{
    // TODO: mTimeStepper is never initialized
}

StepperThread::~StepperThread()
{

}

void StepperThread::start()
{
    mThread = new std::thread(&StepperThread::threadUpdateMethod, this);
}

void StepperThread::setTimeStepper(std::shared_ptr<TimeStepper> timeStepper)
{
    mTimeStepper = timeStepper;
}

TimeStepper* StepperThread::getTimeStepper()
{
    return mTimeStepper.get();
}

void StepperThread::setPaused(bool paused)
{
    mPaused = paused;
}

void StepperThread::addDomain(Domain* domain)
{
    mDomains.push_back(domain);
}

void StepperThread::threadUpdateMethod()
{
    initialization();

    while (mRunning)
    {
        // time stepper saves current time
        mTimeStepper->startStep();
        beforeStep();

        // domains
        for (Domain* domain : mDomains)
        {
            domain->processOperations();
        }

        if (!mPaused)
        {
            step();
        }

        // time stepper calculates time to wait from current time
        // and previous times
        auto sleepingTimeMillis =
                std::chrono::milliseconds(mTimeStepper->finishStep());
        std::this_thread::sleep_for(sleepingTimeMillis);
    }
}
