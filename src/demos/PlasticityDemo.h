#ifndef PLASTICITYDEMO_H
#define PLASTICITYDEMO_H

#include <modules/demo_loader/Demo.h>

class ApplicationControl;

class PlasticityDemo : public Demo
{
public:
    PlasticityDemo(ApplicationControl* ac);

    // Demo interface
public:
    virtual std::string getName();
    virtual void load();
    virtual void unload();

private:

//    void createMobile(const Eigen::Vector& pos, ...)
//    void createMobileRec(SimulationPointRef ref)

    ApplicationControl* mAc;
};

#endif // PLASTICITYDEMO_H
