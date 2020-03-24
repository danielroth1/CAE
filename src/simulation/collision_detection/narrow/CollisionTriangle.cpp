#include "CollisionObjectVisitor.h"
#include "CollisionTriangle.h"

#include <scene/data/geometric/Polygon2DTopology.h>

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

void CollisionTriangle::updateEdgeBoundingBoxes(double collisionMargin)
{
    Polygon2DTopology& topoSource = mAccessor->getTopology2D();
    TopologyFace& face1 = topoSource.getFace(mFaceId);
    for (size_t i = 0; i < 3; ++i)
    {
        if (face1.isEdgeOwner(i))
        {
            TopologyEdge& e1 = topoSource.getEdges()[face1.getEdgeIds()[i]];

            Eigen::Vector& p11 = mAccessor->getPosition(e1.getVertexIds()[0]);
            Eigen::Vector& p12 = mAccessor->getPosition(e1.getVertexIds()[1]);

            mEdgeBBs[i].min() = p11.cwiseMin(p12) - collisionMargin * Eigen::Vector::Ones();
            mEdgeBBs[i].max() = p11.cwiseMax(p12) + collisionMargin * Eigen::Vector::Ones();
        }
    }
}

void CollisionTriangle::update()
{
    // It is not necessary to update the edge AABBs here because we only
    // need to update the ones of faces that are part of the collision.
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
