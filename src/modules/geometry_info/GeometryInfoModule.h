#ifndef GEOMETRYINFOMODULE_H
#define GEOMETRYINFOMODULE_H

#include <modules/Module.h>
#include <memory>

class GeometryInfoUIControl;

class GeometryInfoModule : public Module
{
public:
    GeometryInfoModule();

    // Module interface
public:
    virtual void init(ApplicationControl* ac);
    virtual void initUI(QWidget* parent);
    virtual void finalize();
    virtual std::string getName();
    virtual QWidget* getWidget();

private:
    ApplicationControl* mAc;
    std::shared_ptr<GeometryInfoUIControl> mControl;
};

#endif // GEOMETRYINFOMODULE_H
