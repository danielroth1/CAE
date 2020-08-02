#ifndef CARCRASHDEMO_H
#define CARCRASHDEMO_H

#include <modules/demo_loader/Demo.h>

class ApplicationControl;

// Multiple cars are driving into each other.
// Differnt geometries for deformation simulation, collision detection, and
// visualization are used.
// * Deformation geometry: simple box
// * Collision geometry: convex shape
// * Visualization geometry: FEMFX car chasis model
class CarCrashDemo : public Demo
{
public:
    CarCrashDemo(ApplicationControl* ac);

    // Demo interface
public:
    virtual std::string getName();
    virtual void load();
    virtual void unload();

private:

    ApplicationControl* mAc;
};

#endif // CARCRASHDEMO_H
