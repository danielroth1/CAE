#include "CollisionObjectVisitor.h"
#include "CollisionTriangle.h"

CollisionTriangle::CollisionTriangle(
        const std::shared_ptr<Polygon2DAccessor>& accessor,
        const Face& face,
        ID faceId,
        const std::shared_ptr<SimulationObject>& so)
    : mAccessor(accessor)
    , mFace(face)
    , mFaceId(faceId)
    , mSo(so)
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
