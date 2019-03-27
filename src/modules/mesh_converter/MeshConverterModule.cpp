#include "MeshConverterControl.h"
#include "MeshConverterModule.h"

#include <modules/mesh_converter/ui/MeshConverterUIControl.h>

#include <QWidget>

MeshConverterModule::MeshConverterModule()
{

}

MeshConverterModule::~MeshConverterModule()
{

}

void MeshConverterModule::initUI(QWidget* parent)
{
    mUiControl->init(parent);
}

void MeshConverterModule::init(ApplicationControl* ac)
{
    mControl = std::make_unique<MeshConverterControl>(this, ac);
    mUiControl = std::make_unique<MeshConverterUIControl>(this, ac);

    mControl->init();
}

void MeshConverterModule::finalize()
{
}

std::string MeshConverterModule::getName()
{
    return "Mesh Converter";
}

QWidget* MeshConverterModule::getWidget()
{
    return mUiControl->getWidget();
}

MeshConverterControl* MeshConverterModule::getControl()
{
    return mControl.get();
}

MeshConverterUIControl* MeshConverterModule::getUiControl()
{
    return mUiControl.get();
}
