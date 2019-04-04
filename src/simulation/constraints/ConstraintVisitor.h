#ifndef CONSTRAINTVISITOR_H
#define CONSTRAINTVISITOR_H

// Forward Declarations
class BallJoint;
class CollisionConstraint;
class LinearForce;
class Truncation;

// Visitor for constraints
class ConstraintVisitor
{
public:

    virtual void visit(BallJoint* ballJoint) = 0;

    virtual void visit(CollisionConstraint* cc) = 0;

protected:
    virtual ~ConstraintVisitor();
};

#endif // CONSTRAINTVISITOR_H
