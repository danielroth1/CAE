#ifndef DEMOLOADERUICONTROL_H
#define DEMOLOADERUICONTROL_H


#include <memory>

class Demo;
class DemoLoaderModule;
class DemoLoaderUIForm;
class QWidget;

class DemoLoaderUIControl
{
public:
    DemoLoaderUIControl(
            DemoLoaderModule* mDemoLoaderModule,
            QWidget* parent);

    QWidget* getWidget();

    // Creates and links a corresponding entry in the list view for
    // the given demo. The entry can be removed again by calling
    // removeDemo().
    void addDemo(std::shared_ptr<Demo> demo);

    void removeDemo(std::shared_ptr<Demo> demo);

    void notifyDemoClicked(const std::shared_ptr<Demo>& demo);

private:
    DemoLoaderModule* mDemoLoaderModule;
    DemoLoaderUIForm* mWidget;
};

#endif // DEMOLOADERUICONTROL_H
