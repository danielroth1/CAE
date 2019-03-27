#ifndef MODULE_H
#define MODULE_H

#include <string>

class ApplicationControl;
class QWidget;

class Module
{
public:
    Module();
    virtual ~Module();

    virtual void init(ApplicationControl* ac) = 0;
    virtual void initUI(QWidget* parent) = 0;
    virtual void finalize() = 0;
    virtual std::string getName() = 0;
    virtual QWidget* getWidget() =0;
};

#endif // MODULE_H
