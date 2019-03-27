#ifndef STEPPERTHREAD_H
#define STEPPERTHREAD_H

#include <atomic>
#include <thread>
#include <vector>

class TimeStepper;
class Domain;

// Requires to call initialize() after creation
class StepperThread
{
public:
    StepperThread();
    virtual ~StepperThread();

    void start();

    // Getters/ Setters
    void setTimeStepper(std::shared_ptr<TimeStepper> timeStepper);
    TimeStepper* getTimeStepper();
    void setPaused(bool paused);

    // Delegated Domains
    void addDomain(Domain* domain);
    void removeDomain(Domain* domain);
    void clearDomains();

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

    std::thread* mThread;

    std::shared_ptr<TimeStepper> mTimeStepper;
    std::vector<Domain*> mDomains;
};

#endif // STEPPERTHREAD_H
