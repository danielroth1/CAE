#ifndef SIMULATIONMODULE_H
#define SIMULATIONMODULE_H

#include <modules/Module.h>
#include <memory>

class SimulationUIControl;

class SimulationModule : public Module
{
public:
    SimulationModule();
    virtual ~SimulationModule();

    // Module interface
public:
    virtual void init(ApplicationControl* ac);
    virtual void initUI(QWidget* parent);
    virtual void finalize();
    virtual std::string getName();
    virtual QWidget* getWidget();

    ApplicationControl* mAc;
    std::unique_ptr<SimulationUIControl> mUIControl;
};

#endif // SIMULATIONMODULE_H
