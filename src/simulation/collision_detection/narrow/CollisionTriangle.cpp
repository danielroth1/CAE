#include "CollisionObjectVisitor.h"
#include "CollisionTriangle.h"

CollisionTriangle::CollisionTriangle()
{

}

void CollisionTriangle::accept(CollisionObjectVisitor& visitor)
{
    visitor.visit(this);
}
