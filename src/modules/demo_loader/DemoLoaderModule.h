#ifndef DEMOLOADERMODULE_H
#define DEMOLOADERMODULE_H

#include <memory>
#include <modules/Module.h>

class Demo;
class DemoLoaderUIControl;

class DemoLoaderModule : public Module
{
public:
    DemoLoaderModule();

    // Module interface
public:
    virtual void init(ApplicationControl* ac);
    virtual void initUI(QWidget* parent);
    virtual void finalize();
    virtual std::string getName();
    virtual QWidget* getWidget();

    // Adds a demo to the demo loader.
    // Adds a corresponding entry to the in the list view.
    void addDemo(const std::shared_ptr<Demo>& demo);

    void removeDemo(const std::shared_ptr<Demo>& demo);

    void loadDemo(const std::shared_ptr<Demo>& demo);

private:
    ApplicationControl* mAc;
    std::shared_ptr<DemoLoaderUIControl> mUIControl;
    std::shared_ptr<Demo> mCurrentlyLoadedDemo;
};

#endif // DEMOLOADERMODULE_H
