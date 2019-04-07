#ifndef CONSTRAINT_H
#define CONSTRAINT_H

#include <simulation/MechanicalProperty.h>

// Forward Declarations
class ConstraintVisitor;
class SimulationObject;

class Constraint : public MechanicalProperty
{
public:
    Constraint();

    virtual void initialize() = 0;

    virtual bool solve(double maxConstraintError) = 0;

    virtual void accept(ConstraintVisitor& cv) = 0;

protected:
    virtual ~Constraint();

};

#endif // CONSTRAINT_H
