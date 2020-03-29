#include "CollisionObjectVisitor.h"
#include "CollisionTriangle.h"

#include <scene/data/geometric/Polygon2DTopology.h>

#include <iostream>

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
            const TopologyEdge& e1 = topoSource.getEdges()[face1.getEdgeIds()[i]];

            const Eigen::Vector& p1 = mAccessor->getPosition(e1.getVertexIds()[0]);
            const Eigen::Vector& p2 = mAccessor->getPosition(e1.getVertexIds()[1]);
#ifdef USE_EDGE_BV_STRATEGY_AABB
            // AABB update
            mEdgeBBs[i].min() = p1.cwiseMin(p2) - collisionMargin * Eigen::Vector::Ones();
            mEdgeBBs[i].max() = p1.cwiseMax(p2) + collisionMargin * Eigen::Vector::Ones();

#elif USE_EDGE_BV_STRATEGY_OBB
            // OBB update
            Eigen::Vector dir = (p2 - p1);
            double length = dir.norm();
            dir /= length;

            mEdgeBBs[i].getPosition() = p1 + 0.5 * length * dir;
            mEdgeBBs[i].getAxes()[0] = dir;

            // calculate 2 arbitrary orthogonal vectors
            if (std::abs(dir(1)) > 1e-3)
                mEdgeBBs[i].getAxes()[1] = Eigen::Vector3d(0, dir(2), -dir(1));
            else if (std::abs(dir(0)) > 1e-3)
                mEdgeBBs[i].getAxes()[1] = Eigen::Vector3d(-dir(1), dir(0), 0);
            else
                mEdgeBBs[i].getAxes()[1] = Eigen::Vector3d(-dir(2), 0, dir(0));
            mEdgeBBs[i].getAxes()[1].normalize();

            mEdgeBBs[i].getAxes()[2] = dir.cross(mEdgeBBs[i].getAxes()[1]);
            mEdgeBBs[i].getHalfSizes()[0] = 0.5 * length;
            mEdgeBBs[i].getHalfSizes()[1] = collisionMargin;
            mEdgeBBs[i].getHalfSizes()[2] = collisionMargin;

#elif USE_EDGE_BV_STRATEGY_ORIENTED_AABB
            // Rotated AABB update
            Eigen::Vector3d v = p2 - p1;
            double length = v.norm();

            Eigen::Quaterniond q(
                        length * length + length * v(1), -length * v(2), 0, length * v(0));
            mEdgeBBs[i].setRotation(q.normalized());

            mEdgeBBs[i].setWidth(2 * collisionMargin);
            mEdgeBBs[i].setHeight(length);
            mEdgeBBs[i].setTranslation(p1);
#endif
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
