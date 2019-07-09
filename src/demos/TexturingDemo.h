#ifndef TEXTURINGDEMO_H
#define TEXTURINGDEMO_H

#include <modules/demo_loader/Demo.h>

class ApplicationControl;

class TexturingDemo : public Demo
{
public:
    TexturingDemo(ApplicationControl* ac);

    // Demo interface
public:
    virtual std::string getName();
    virtual void load();
    virtual void unload();

private:
    ApplicationControl* mAc;
};

#endif // TEXTURINGDEMO_H
