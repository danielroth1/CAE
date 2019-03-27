#ifndef CONSTRAINT_H
#define CONSTRAINT_H

#include <memory>

// Forward Declarations
class ConstraintVisitor;
class SimulationObject;

class Constraint : public std::enable_shared_from_this<Constraint>
{
public:
    Constraint();

    virtual void accept(ConstraintVisitor& cv) = 0;

    virtual bool references(Constraint* c) = 0;
    virtual bool references(SimulationObject* so) = 0;

protected:
    virtual ~Constraint();

};

#endif // CONSTRAINT_H
