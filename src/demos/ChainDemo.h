#ifndef CHAINDEMO_H
#define CHAINDEMO_H

#include <modules/demo_loader/Demo.h>

class ApplicationControl;

class ChainDemo : public Demo
{
public:
    ChainDemo(ApplicationControl& ac);

    // Demo interface
public:
    virtual std::string getName();
    virtual void load();
    virtual void unload();

private:
    ApplicationControl& mAc;
};

#endif // CHAINDEMO_H
