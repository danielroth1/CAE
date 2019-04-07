#ifndef BALLJOINTDEMO_H
#define BALLJOINTDEMO_H

#include <modules/demo_loader/Demo.h>

class ApplicationControl;

class BallJointDemo : public Demo
{
public:
    BallJointDemo(ApplicationControl& ac);

    // Demo interface
public:
    virtual std::string getName();
    virtual void load();
    virtual void unload();

private:
    ApplicationControl& mAc;
};

#endif // BALLJOINTDEMO_H
