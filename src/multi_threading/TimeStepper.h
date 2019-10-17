#ifndef TIMESTEPPER_H
#define TIMESTEPPER_H

// Is used in Thread.
// Calculates waiting time between each each cycle in the while loop.
// Call startStep() at the start and finishStep() at the end of
// each step.
class TimeStepper
{
public:
    // \param sleepTime - used sleep time in micro seconds:
    TimeStepper(int sleepTime);
    virtual ~TimeStepper();

    // Resets the clock. Can be called at any point.
    virtual void reset();

    // Prepare calculation for sleeping time.
    virtual void startStep();

    // Returns number of microseconds to sleep.
    virtual int finishStep();

private:
    int mSleepTime;
};

#endif // TIMESTEPPER_H
