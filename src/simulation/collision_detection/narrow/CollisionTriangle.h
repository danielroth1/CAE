#ifndef COLLISIONTRIANGLE_H
#define COLLISIONTRIANGLE_H

#include "CollisionObject.h"

#include <scene/data/geometric/MeshInterpolatorFEM.h>
#include <scene/data/geometric/Polygon2DAccessor.h>

#include <data_structures/BoundingBox.h>

class SimulationObject;

// TODO: A collision triangle shouldn't store a reference to the simulation object
// (rigid or deformable) because of the additional memory consumption.
// global parameters:
//      Polygon2DAccessor
// parameters:
//      triangle id
class CollisionTriangle : public CollisionObject
{
public:
    CollisionTriangle(
            const std::shared_ptr<Polygon2DAccessor>& accessor,
            const Face& face,
            ID faceId);

    virtual ~CollisionTriangle() override;

    const std::shared_ptr<Polygon2DAccessor>& getAccessor() const
    {
        return mAccessor;
    }

    const Face& getFace()
    {
        return mFace;
    }

    ID getFaceId() const
    {
        return mFaceId;
    }

    // \param index - number from 0 to 2
    Eigen::Vector& getPosition(ID index)
    {
        return mAccessor->getPosition(mFace[index]);
    }

    Eigen::Vector& getP1()
    {
        return mAccessor->getPosition(mFace[0]);
    }

    Eigen::Vector& getP2()
    {
        return mAccessor->getPosition(mFace[1]);
    }

    Eigen::Vector& getP3()
    {
        return mAccessor->getPosition(mFace[2]);
    }

    const std::array<BoundingBox, 3>& getEdgeBoundingBoxes() const
    {
        return mEdgeBBs;
    }

    // Updates the edge AABBs. There is one AABB for each edge that is
    // owned by this triangle. AABBs of non-owned edges are left empty.
    // Access the update AABBs with getEdgeBoundingBoxes(). Call this method
    // before performing the triangle-triangle collision detection.
    void updateEdgeBoundingBoxes(double collisionMargin);

    // CollisionObject interface
public:
    virtual void update() override;
    virtual void updatePrevious() override;
    virtual Type getType() const override;
    virtual void accept(CollisionObjectVisitor& visitor) override;
    // Not used.
    virtual Eigen::Vector getPosition() override;

private:

    std::shared_ptr<Polygon2DAccessor> mAccessor;

    Face mFace;

    std::array<BoundingBox, 3> mEdgeBBs;

    ID mFaceId;
};

#endif // COLLISIONTRIANGLE_H
