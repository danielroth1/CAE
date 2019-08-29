#include "ConstraintSolver.h"

#include <simulation/constraints/ConstraintVisitor.h>

ConstraintSolver::ConstraintSolver()
{

}

ConstraintSolver::~ConstraintSolver()
{

}

void ConstraintSolver::solveConstraints(
        int maxIterations,
        double maxConstraintError)
{
    size_t validConstraints = 0;
    size_t totalConstraints = mConstraints.size() + mCollisionConstraints.size();

    int iterCount = 0;
    for (int iter = 0; iter < maxIterations; ++iter)
    {
        ++iterCount;

        // iterate non-collision constraints
        for (size_t i = 0; i < mConstraints.size(); ++i)
        {
            if (validConstraints == totalConstraints)
                break;

            if (mConstraints[i]->solve(maxConstraintError))
            {
                // if constraint is valid, nothing to do
                ++validConstraints;
            }
            else
            {
                // if not, this corrected coinstraint will be the only one
                // we can be sure about to be valid since the correction
                // can make any other constraint invalid again.
                validConstraints = 1;
            }
        }

        // iterate collisions for as long as each u rel is equal to target u rel
        for (size_t i = 0; i < mCollisionConstraints.size(); ++i)
        {
            if (validConstraints == totalConstraints)
                break;

            if (mCollisionConstraints[i].solve(maxConstraintError))
            {
                // if constraint is valid, nothing to do
                ++validConstraints;
            }
            else
            {
                // if not, this corrected coinstraint will be the only one
                // we can be sure about to be valid since the correction
                // can make any other constraint invalid again.
                validConstraints = 1;
            }
        }
        if (validConstraints == totalConstraints)
            break;
    }
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

std::vector<std::shared_ptr<Constraint>>& ConstraintSolver::getConstraints()
{
    return mConstraints;
}

std::vector<CollisionConstraint>& ConstraintSolver::getCollisionConstraints()
{
    return mCollisionConstraints;
}

