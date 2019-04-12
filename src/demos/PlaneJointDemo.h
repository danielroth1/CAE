#ifndef PLANEJOINTDEMO_H
#define PLANEJOINTDEMO_H

#include <modules/demo_loader/Demo.h>

class ApplicationControl;

class PlaneJointDemo : public Demo
{
public:
    PlaneJointDemo(ApplicationControl& ac);

    // Demo interface
public:
    virtual std::string getName();
    virtual void load();
    virtual void unload();

private:
    ApplicationControl& mAc;
};

#endif // PLANEJOINTDEMO_H
