#ifndef CARDEMO2_H
#define CARDEMO2_H

#include <data_structures/DataStructures.h>
#include <modules/demo_loader/Demo.h>
#include <scene/scene_graph/SGCore.h>

#include <string>

class ApplicationControl;
class SGControl;
class SimulationPointRef;

// Highly tesselated car. Not completely implemented yet.
//
class CarDemo2 : public Demo
{
public:
    CarDemo2(ApplicationControl& ac);

    // Demo interface
public:
    virtual std::string getName();
    virtual void load();
    virtual void unload();

private:

    ApplicationControl& mAc;
    SGControl* mSg;
};

#endif // CARDEMO2_H
