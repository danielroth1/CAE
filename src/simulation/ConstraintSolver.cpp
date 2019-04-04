#include "ConstraintSolver.h"

#include <simulation/constraints/ConstraintVisitor.h>

ConstraintSolver::ConstraintSolver()
    : mConstraintDispatcher(*this)
{

}

ConstraintSolver::~ConstraintSolver()
{

}

void ConstraintSolver::solveConstraints(
        int maxIterations,
        double maxConstraintError)
{
    for (int i = 0; i < maxIterations; ++i)
    {
        for (const std::shared_ptr<Constraint>& c : mConstraints)
        {
            solveConstraint(c, maxConstraintError);
        }
        for (CollisionConstraint& cc : mCollisionConstraints)
        {
            solveConstraint(cc, maxConstraintError);
        }
    }
}

bool ConstraintSolver::solveConstraint(
        const std::shared_ptr<Constraint>& c,
        double maxConstraintError)
{
    mConstraintDispatcher.maxConstraintError = maxConstraintError;
    c->accept(mConstraintDispatcher);
    return mConstraintDispatcher.returnValue;
}

void ConstraintSolver::addConstraint(const std::shared_ptr<Constraint>& constraint)
{
    mConstraints.push_back(constraint);
}

void ConstraintSolver::removeConstraint(const std::shared_ptr<Constraint>& constraint)
{
    auto it = std::find(mConstraints.begin(), mConstraints.end(), constraint);
    if (it != mConstraints.end())
    {
        mConstraints.erase(it);
    }
}

void ConstraintSolver::addCollisionConstraint(CollisionConstraint& cc)
{
    mCollisionConstraints.push_back(cc);
}

void ConstraintSolver::clearCollisionConstraints()
{
    mCollisionConstraints.clear();
}

void ConstraintSolver::reserveCollisionConstraint(size_t size)
{
    mCollisionConstraints.reserve(size);
}

