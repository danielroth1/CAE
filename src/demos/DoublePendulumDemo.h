#ifndef DOUBLEPENDULUMDEMO_H
#define DOUBLEPENDULUMDEMO_H

#include <modules/demo_loader/Demo.h>

class ApplicationControl;

class DoublePendulumDemo : public Demo
{
public:
    DoublePendulumDemo(ApplicationControl& ac);

    // Demo interface
public:
    virtual std::string getName();
    virtual void load();
    virtual void unload();

private:
    ApplicationControl& mAc;
};

#endif // DoublePendulumDemo
