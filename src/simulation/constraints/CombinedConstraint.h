#ifndef COMBINEDCONSTRAINT_H
#define COMBINEDCONSTRAINT_H

#include "Constraint.h"

#include <vector>

// A combination of multiple constraints
class CombinedConstraint : public Constraint
{
public:
    CombinedConstraint();

    // MechanicalProperty interface
public:
    virtual bool references(const std::shared_ptr<SimulationObject>& so);

    // Constraint interface
public:
    virtual void initialize(double stepSize);
    virtual bool solve(double maxConstraintError);
    virtual void accept(ConstraintVisitor& cv);

protected:
    void addConstraint(const std::shared_ptr<Constraint>& c);
    void removeConstraint(const std::shared_ptr<Constraint>& c);

private:
    std::vector<std::shared_ptr<Constraint>> mConstraints;
};

#endif // COMBINEDCONSTRAINT_H
