#ifndef FIXEDROTATIONALJOINTDEMO_H
#define FIXEDROTATIONALJOINTDEMO_H

#include <modules/demo_loader/Demo.h>

class ApplicationControl;

class FixedRotationalJointDemo : public Demo
{
public:
    FixedRotationalJointDemo(ApplicationControl& ac);

    // Demo interface
public:
    virtual std::string getName();
    virtual void load();
    virtual void unload();

private:
    ApplicationControl& mAc;
};

#endif // FIXEDROTATIONALJOINTDEMO_H
