#ifndef JOINTTYPESDEMO_H
#define JOINTTYPESDEMO_H

#include <modules/demo_loader/Demo.h>

class ApplicationControl;

class JointTypesDemo : public Demo
{
public:
    JointTypesDemo(ApplicationControl& ac);

    // Demo interface
public:
    virtual std::string getName();
    virtual void load();
    virtual void unload();

private:
    ApplicationControl& mAc;
};

#endif // JOINTTYPESDEMO_H
