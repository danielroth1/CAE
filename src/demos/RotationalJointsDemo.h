#ifndef ROTATIONALJOINTSDEMO_H
#define ROTATIONALJOINTSDEMO_H

#include <modules/demo_loader/Demo.h>

class ApplicationControl;

class RotationalJointsDemo : public Demo
{
public:
    RotationalJointsDemo(ApplicationControl& ac);

    // Demo interface
public:
    virtual std::string getName();
    virtual void load();
    virtual void unload();

private:
    ApplicationControl& mAc;
};

#endif // ROTATIONALJOINTSDEMO_H
