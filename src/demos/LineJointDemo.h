#ifndef LINEJOINTDEMO_H
#define LINEJOINTDEMO_H

#include <modules/demo_loader/Demo.h>

class ApplicationControl;

class LineJointDemo : public Demo
{
public:
    LineJointDemo(ApplicationControl& ac);

    // Demo interface
public:
    virtual std::string getName();
    virtual void load();
    virtual void unload();

private:
    ApplicationControl& mAc;
};

#endif // LINEJOINTDEMO_H
