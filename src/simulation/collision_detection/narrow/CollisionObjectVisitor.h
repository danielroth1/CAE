#ifndef COLLISIONOBJECTVISITOR_H
#define COLLISIONOBJECTVISITOR_H


class CollisionSphere;
class CollisionTriangle;

class CollisionObjectVisitor
{
public:

    virtual void visit(CollisionSphere* collisionSphere) = 0;

    virtual void visit(CollisionTriangle* collisionTriangle) = 0;

protected:
    CollisionObjectVisitor();
    virtual ~CollisionObjectVisitor();
};

#endif // COLLISIONOBJECTVISITOR_H
