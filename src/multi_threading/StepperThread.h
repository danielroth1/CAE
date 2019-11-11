#ifndef STEPPERTHREAD_H
#define STEPPERTHREAD_H

#include <atomic>
#include <thread>
#include <vector>

class TimeStepper;
class Domain;

// A Stepper thread is a thread that supports Domains.
// It iterates a single while loop.
// At the start of each iteration, all operations of all domains are executed.
// A TimeStepper determines the sleeping time after each iteration.
// Call start() to start the thread.
class StepperThread
{
public:
    StepperThread();
    virtual ~StepperThread();

    // Creats and starts the thread. After calling this method, operations in the
    // domains operation queue are processed. Sets the threads id to all domains.
    void start();

    // Getters/ Setters
    void setTimeStepper(std::shared_ptr<TimeStepper> timeStepper);
    TimeStepper* getTimeStepper();
    void setPaused(bool paused);

    void performSingleStep();

    // Delegated Domains
    // Either sets the domains threads id now or if not already done
    // when start() is executed.
    void addDomain(Domain* domain);

    // Stepper thread methods
    virtual void initialization() = 0;

    // This is called before processing operations.
    virtual void beforeStep() = 0;

    // This is called after processing operations.
    virtual void step() = 0;

private:

    //
    void threadUpdateMethod();

    std::atomic<bool> mRunning { true };
    std::atomic<bool> mPaused { false };

    std::shared_ptr<std::thread> mThread;

    std::shared_ptr<TimeStepper> mTimeStepper;
    std::vector<Domain*> mDomains;
};

#endif // STEPPERTHREAD_H
