#ifndef CONSTRAINTSOLVER_H
#define CONSTRAINTSOLVER_H

#include <simulation/constraints/CollisionConstraint.h>
#include <simulation/constraints/ConstraintVisitor.h>

#include <vector>

class BallJoint;

// TODO: how to incorporate collision constraints? without memory allocations?
// impelemnt ImpulseConstraintSolver
// BallJointConstraints?
class ConstraintSolver
{
public:
    ConstraintSolver();

    virtual ~ConstraintSolver();

    // Solves all constraints
    virtual void solveConstraints(
            int maxIterations,
            double maxConstraintError);

    // Adds a constraint. Doesn't check if the constraint was already added.
    void addConstraint(const std::shared_ptr<Constraint>& constraint);
    void removeConstraint(const std::shared_ptr<Constraint>& constraint);

    void addCollisionConstraint(CollisionConstraint& cc);
    void clearCollisionConstraints();
    void reserveCollisionConstraint(size_t size);

    std::vector<std::shared_ptr<Constraint>>& getConstraints();
    std::vector<CollisionConstraint>& getCollisionConstraints();

protected:
    // All non collision constraints.
    std::vector<std::shared_ptr<Constraint>> mConstraints;

    // All collision constraints.
    // Because at every iterations, collision constraints
    // are added/ removed, they are saved sepearatly wihtout
    // using pointers and additional heap memory allocations.
    std::vector<CollisionConstraint> mCollisionConstraints;
};

#endif // CONSTRAINTSOLVER_H
