#include "SimulationModule.h"

#include <ApplicationControl.h>
#include <QWidget>

#include <simulation/ui/SimulationUIControl.h>

SimulationModule::SimulationModule()
{

}

SimulationModule::~SimulationModule()
{

}

void SimulationModule::init(ApplicationControl* ac)
{
    mAc = ac;
    mUIControl = std::make_unique<SimulationUIControl>(this, mAc);
}

void SimulationModule::initUI(QWidget* parent)
{
    mUIControl->init(parent);
}

void SimulationModule::finalize()
{

}

std::string SimulationModule::getName()
{
    return "Simulation";
}

QWidget* SimulationModule::getWidget()
{
    return mUIControl->getWidget();
}
