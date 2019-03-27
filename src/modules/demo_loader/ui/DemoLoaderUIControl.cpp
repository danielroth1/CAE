#include "DemoLoaderUIControl.h"
#include "DemoLoaderUIForm.h"

#include <modules/demo_loader/Demo.h>
#include <modules/demo_loader/DemoLoaderModule.h>

DemoLoaderUIControl::DemoLoaderUIControl(
        DemoLoaderModule* demoLoaderModule,
        QWidget* parent)
    : mDemoLoaderModule(demoLoaderModule)
{
    mWidget = new DemoLoaderUIForm(this, parent);
}

QWidget* DemoLoaderUIControl::getWidget()
{
    return mWidget;
}

void DemoLoaderUIControl::addDemo(std::shared_ptr<Demo> demo)
{
    mWidget->addDemo(demo);
}

void DemoLoaderUIControl::removeDemo(std::shared_ptr<Demo> demo)
{
    mWidget->removeDemo(demo);
}

void DemoLoaderUIControl::notifyDemoClicked(const std::shared_ptr<Demo>& demo)
{
    mDemoLoaderModule->loadDemo(demo);
}
