#ifndef BOUNDINGBOX_H
#define BOUNDINGBOX_H

#include "data_structures/DataStructures.h"

using namespace Eigen;

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
