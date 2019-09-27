#ifndef CUBEWALLDEMO_H
#define CUBEWALLDEMO_H

#include <modules/demo_loader/Demo.h>

class ApplicationControl;

class CubeWallDemo : public Demo
{
public:
    CubeWallDemo(ApplicationControl* ac,
                 bool rigid);

    // Demo interface
public:
    virtual std::string getName();
    virtual void load();
    virtual void unload();

private:
    ApplicationControl* mAc;

    std::string mName;
    bool mRigid;
};

#endif // CUBEWALLDEMO_H
