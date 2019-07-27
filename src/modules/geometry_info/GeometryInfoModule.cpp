#include "GeometryInfoModule.h"

#include <modules/geometry_info/ui/GeometryInfoUIControl.h>
#include <modules/geometry_info/ui/GeometryInfoUIForm.h>

#include <ApplicationControl.h>

#include <ui/UIControl.h>

GeometryInfoModule::GeometryInfoModule()
{

}

void GeometryInfoModule::init(ApplicationControl* ac)
{
    mAc = ac;
}

void GeometryInfoModule::initUI(QWidget* parent)
{
    mControl = std::make_shared<GeometryInfoUIControl>(
                mAc->getUIControl()->getSelectionControl());

    mControl->init(parent);
}

void GeometryInfoModule::finalize()
{
    mControl->finalize();
}

std::string GeometryInfoModule::getName()
{
    return "Geometry Info";
}

QWidget* GeometryInfoModule::getWidget()
{
    return mControl->getWidget();
}
