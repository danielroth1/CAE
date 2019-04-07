#ifndef TIMESTEPPER_H
#define TIMESTEPPER_H

// Is used in Thread.
// Calculates waiting time between each each cycle in the while loop.
// Call startStep() at the start and finishStep() at the end of
// each step.
class TimeStepper
{
public:
    TimeStepper(int sleepTime);

    // Resets the clock. Can be called at any point.
    void reset();

    // Prepare calculation for sleeping time.
    void startStep();

    // Returns number of milliseconds to sleep.
    int finishStep();

private:
    int mSleepTime;
};

#endif // TIMESTEPPER_H
