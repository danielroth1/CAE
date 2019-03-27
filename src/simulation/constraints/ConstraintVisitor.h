#ifndef CONSTRAINTVISITOR_H
#define CONSTRAINTVISITOR_H

// Forward Declarations
class LinearForce;
class Truncation;

// Visitor for constraints
class ConstraintVisitor
{
public:

    virtual void visit(LinearForce* linearForce) = 0;

protected:
    virtual ~ConstraintVisitor();
};

#endif // CONSTRAINTVISITOR_H
