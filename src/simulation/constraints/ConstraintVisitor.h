#ifndef CONSTRAINTVISITOR_H
#define CONSTRAINTVISITOR_H

// Forward Declarations
class BallJoint;
class CollisionConstraint;
class DistanceJoint;
class DoubleAxisRotationalJoint;
class FixedRotationalJoint;
class HingeJoint;
class LineJoint;
class PlaneJoint;
class LinearForce;
class Truncation;

// Visitor for constraints
class ConstraintVisitor
{
public:

    virtual void visit(BallJoint* ballJoint) = 0;

    virtual void visit(CollisionConstraint* cc) = 0;

    virtual void visit(DistanceJoint* joint) = 0;

    virtual void visit(DoubleAxisRotationalJoint* joint) = 0;

    virtual void visit(FixedRotationalJoint* joint) = 0;

    virtual void visit(HingeJoint* joint) = 0;

    virtual void visit(LineJoint* joint) = 0;

    virtual void visit(PlaneJoint* joint) = 0;

protected:
    virtual ~ConstraintVisitor();
};

#endif // CONSTRAINTVISITOR_H
