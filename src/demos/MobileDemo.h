#ifndef MOBILEDEMO_H
#define MOBILEDEMO_H

#include <modules/demo_loader/Demo.h>

class ApplicationControl;

class MobileDemo : public Demo
{
public:
    MobileDemo(ApplicationControl& ac);

    // Demo interface
public:
    virtual std::string getName();
    virtual void load();
    virtual void unload();

private:

//    void createMobile(const Eigen::Vector& pos, ...)
//    void createMobileRec(SimulationPointRef ref)

    ApplicationControl& mAc;
};

#endif // MOBILEDEMO_H
