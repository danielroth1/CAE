#ifndef CONSTRAINEDDEFORMABLEDEMO_H
#define CONSTRAINEDDEFORMABLEDEMO_H

#include <modules/demo_loader/Demo.h>


class ApplicationControl;

class ConstrainedDeformableDemo : public Demo
{
public:
    ConstrainedDeformableDemo(ApplicationControl* ac);

    // Demo interface
public:
    std::string getName();
    void load();
    void unload();

private:

    ApplicationControl* mAc;

};

#endif // CONSTRAINEDDEFORMABLEDEMO_H
