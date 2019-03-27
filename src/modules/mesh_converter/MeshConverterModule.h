#ifndef MESHCONVERTERMODULE_H
#define MESHCONVERTERMODULE_H

#include <memory>
#include <modules/Module.h>

class MeshConverterControl;
class MeshConverterUIControl;
class QWidget;

class MeshConverterModule : public Module
{
public:
    MeshConverterModule();
    virtual ~MeshConverterModule();

    // Module interface
public:
    virtual void init(ApplicationControl* ac);
    virtual void initUI(QWidget* parent);
    virtual void finalize();
    virtual std::string getName();
    virtual QWidget* getWidget();

    MeshConverterControl* getControl();
    MeshConverterUIControl* getUiControl();

private:
    std::unique_ptr<MeshConverterControl> mControl;
    std::unique_ptr<MeshConverterUIControl> mUiControl;
};

#endif // MESHCONVERTERMODULE_H
