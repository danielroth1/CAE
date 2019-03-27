#ifndef BOUNDINGVOLUMEVISITOR_H
#define BOUNDINGVOLUMEVISITOR_H

class BVSphere;

class BoundingVolumeVisitor
{
public:
    virtual void visit(BVSphere* sphere) = 0;

protected:
    BoundingVolumeVisitor();
    virtual ~BoundingVolumeVisitor();
};

#endif // BOUNDINGVOLUMEVISITOR_H
