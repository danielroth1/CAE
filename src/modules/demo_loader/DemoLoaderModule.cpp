#include "Demo.h"
#include "DemoLoaderModule.h"

#include <modules/demo_loader/ui/DemoLoaderUIControl.h>

#include <ApplicationControl.h>

#include <ui/UIControl.h>

DemoLoaderModule::DemoLoaderModule()
{
}

void DemoLoaderModule::init(ApplicationControl* ac)
{
    mAc = ac;
}

void DemoLoaderModule::initUI(QWidget* parent)
{
    mUIControl = std::make_shared<DemoLoaderUIControl>(this, parent);
}

void DemoLoaderModule::finalize()
{

}

std::string DemoLoaderModule::getName()
{
    return "Demo Loader";
}

QWidget*DemoLoaderModule::getWidget()
{
    return mUIControl->getWidget();
}

void DemoLoaderModule::addDemo(const std::shared_ptr<Demo>& demo)
{
    mUIControl->addDemo(demo);
}

void DemoLoaderModule::removeDemo(const std::shared_ptr<Demo>& demo)
{
    mUIControl->removeDemo(demo);
}

void DemoLoaderModule::loadDemo(const std::shared_ptr<Demo>& demo)
{
    // restore default parameters
    mAc->getSimulationControl()->setGravity(Eigen::Vector(0.0, -9.81, 0.0));
    mAc->getSimulationControl()->setStepSize(0.01);

    mAc->getSGControl()->clearScene();
    if (mCurrentlyLoadedDemo)
        mCurrentlyLoadedDemo->unload();

    mCurrentlyLoadedDemo = demo;
    mCurrentlyLoadedDemo->load();
    mAc->getUIControl()->revalidateTreeWidget();
}
