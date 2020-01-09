#include "CollisionObjectVisitor.h"
#include "CollisionTriangle.h"

CollisionTriangle::CollisionTriangle(
        const std::shared_ptr<Polygon2DAccessor>& accessor,
        const Face& face,
        ID faceId)
    : mAccessor(accessor)
    , mFace(face)
    , mFaceId(faceId)
{

}

CollisionTriangle::~CollisionTriangle()
{

}

void CollisionTriangle::update()
{

}

void CollisionTriangle::updatePrevious()
{

}

CollisionObject::Type CollisionTriangle::getType() const
{
    return CollisionObject::Type::TRIANGLE;
}

void CollisionTriangle::accept(CollisionObjectVisitor& visitor)
{
    visitor.visit(this);
}

Eigen::Vector CollisionTriangle::getPosition()
{
    return Eigen::Vector::Zero();
}
