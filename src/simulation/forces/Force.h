#ifndef FORCE_H
#define FORCE_H

#include <simulation/MechanicalProperty.h>



class Force : public MechanicalProperty
{
public:
    Force();
    virtual ~Force();

    virtual void applyForce() = 0;
};

#endif // FORCE_H
