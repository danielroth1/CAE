#ifndef INTERPOLATORCREATIONDEMO_H
#define INTERPOLATORCREATIONDEMO_H

#include <modules/demo_loader/Demo.h>

class ApplicationControl;

// Demo to test the InterpolatorModule for the creation of Interpolators via
// the UI.
// Creates two times a sphere that is inside of a box next to each other.
// The box is a Polygon3D and a FemObject. There is no gravity.
class InterpolatorCreationDemo : public Demo
{
public:
    InterpolatorCreationDemo(ApplicationControl* ac);

    // Demo interface
public:
    virtual std::string getName();
    virtual void load();
    virtual void unload();

private:

    ApplicationControl* mAc;
};

#endif // INTERPOLATORCREATIONDEMO_H
