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

    // Dispatches the constraint to one of the other solverConstraint
    // methods.
    // \return if the maxConstraintError could be satisfied
    virtual bool solveConstraint(
            const std::shared_ptr<Constraint>& c,
            double maxConstraintError);

    // \return if the maxConstraintError could be satisfied
    virtual bool solveConstraint(
            CollisionConstraint& cc,
            double maxConstraintError) = 0;

    virtual bool solveConstraint(
            BallJoint& ballJoint,
            double maxConstraintError) = 0;

    // Adds a constraint. Doesn't check if the constraint was already added.
    void addConstraint(const std::shared_ptr<Constraint>& constraint);
    void removeConstraint(const std::shared_ptr<Constraint>& constraint);

    void addCollisionConstraint(CollisionConstraint& cc);
    void clearCollisionConstraints();
    void reserveCollisionConstraint(size_t size);

protected:
    // All non collision constraints.
    std::vector<std::shared_ptr<Constraint>> mConstraints;

    // All collision constraints.
    // Because at every iterations, collision constraints
    // are added/ removed, they are saved sepearatly wihtout
    // using pointers and additional heap memory allocations.
    std::vector<CollisionConstraint> mCollisionConstraints;

private:
    // Dispatch the constraints
    class ConstraintDispatcher : public ConstraintVisitor
    {
    public:
        ConstraintDispatcher(ConstraintSolver& _cs)
            : cs(_cs)
        {

        }

        virtual void visit(BallJoint* ballJoint)
        {
            returnValue = cs.solveConstraint(*ballJoint, maxConstraintError);
        }

        virtual void visit(CollisionConstraint* /*cc*/)
        {
            returnValue = false;
            // This is done
        }

        ConstraintSolver& cs;
        double maxConstraintError;
        bool returnValue;
    };

    ConstraintDispatcher mConstraintDispatcher;
};

#endif // CONSTRAINTSOLVER_H
