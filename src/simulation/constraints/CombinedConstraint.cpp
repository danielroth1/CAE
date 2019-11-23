#include "CombinedConstraint.h"

#include <algorithm>

CombinedConstraint::CombinedConstraint()
{

}

bool CombinedConstraint::references(SimulationObject* so)
{
    for (const std::shared_ptr<Constraint>& c : mConstraints)
    {
        if (c->references(so))
            return true;
    }
    return false;
}

void CombinedConstraint::initialize(double stepSize)
{
    for (const std::shared_ptr<Constraint>& c : mConstraints)
    {
        c->initialize(stepSize);
    }
}

bool CombinedConstraint::solve(double maxConstraintError)
{
    bool resolved = true;
    for (const std::shared_ptr<Constraint>& c : mConstraints)
    {
        resolved &= c->solve(maxConstraintError);
    }
    return resolved;
}

void CombinedConstraint::addConstraint(const std::shared_ptr<Constraint>& c)
{
    mConstraints.push_back(c);
}

void CombinedConstraint::removeConstraint(const std::shared_ptr<Constraint>& c)
{
    auto it = std::find(mConstraints.begin(), mConstraints.end(), c);
    if (it != mConstraints.end())
    {
        mConstraints.erase(it);
    }
}
