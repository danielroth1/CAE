#ifndef BOUNDINGBOX_H
#define BOUNDINGBOX_H

#include "data_structures/DataStructures.h"

using namespace Eigen;

// This class represents an Axis Aligned Bounding Box (AABB).
// A AABB is a box that is aligned by the x-, y-, and z-axis, so all its sides
// are parallel to these axes.
// AABBs offer a quick update routine and intersection tests.
// Their problem is that they don't approximate objects as nicely as for example
// OrientedBoundingBoxes (OBB).
// Another issue compared to OBB is that they are based on vertex positions
// alone so they always need to be updated if a vertex position changed.
class BoundingBox
{
public:
    BoundingBox();

    // Checks if the bounding box is intersecting with the given bounding
    // box. This is the case if any planes are intersecting or one bounding
    // box is inside the other one.
    bool intersects(const BoundingBox& bb) const
    {
        return mMin(0) < bb.max()(0) && bb.min()(0) < mMax(0) &&
                mMin(1) < bb.max()(1) && bb.min()(1) < mMax(1) &&
                mMin(2) < bb.max()(2) && bb.min()(2) < mMax(2);
    }

    // Checks if the given point is inside the bounding box.
    bool isInside(const Eigen::Vector3d& point)
    {
        return mMin(0) < point(0) && point(0) < mMax(0) &&
                mMin(1) < point(1) && point(1) < mMax(1) &&
                mMin(2) < point(2) && point(2) < mMax(2);
    }

    // Checks if the given point is inside the bounding box which is extended
    // by the given margin.
    bool isInside(const Eigen::Vector3d& point, double margin)
    {
        return mMin(0) - margin < point(0) && point(0) < mMax(0) + margin &&
                mMin(1) - margin < point(1) && point(1) < mMax(1) + margin &&
                mMin(2) - margin < point(2) && point(2) < mMax(2) + margin;
    }

    Vector& min()
    {
        return mMin;
    }

    const Vector& min() const
    {
        return mMin;
    }

    Vector& max()
    {
        return mMax;
    }

    const Vector& max() const
    {
        return mMax;
    }

    Vector& mid()
    {
        return mMid;
    }

    const Vector& mid() const
    {
        return mMid;
    }

    Vector& size()
    {
        return mSize;
    }

    const Vector& size() const
    {
        return mSize;
    }


private:
    Vector mMin;
    Vector mMax;
    Vector mMid;
    Vector mSize;
};

#endif // BOUNDINGBOX_H
