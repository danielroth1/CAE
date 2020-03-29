#ifndef ORIENTEDAABB_H
#define ORIENTEDAABB_H

#include <Eigen/Dense>


// This is a special bounding volume.
// The idea is to have a structure with a faster intersection test than
// OBBs and a better object approximation than AABBs.
// The box itself is a OBB so it stores rotation and translation. It stores
// the width in x and z direction and the height in y direction. Theoretically,
// different values for x and z could be used but this wasn't necessary in the
// scenario where this box is used.
// The difference to a OBB is the intersection test: It only checks for an
// intersection with another edge. The edge points are rotated / translated so
// that the OBB lies unrotated in the origin (edge is transformed in the space
// of the OBB). Then a AABB for the transformed edge points is calculated.
// Finally, a AABB check between the unrotated OBB (which is a AABB) and the
// edge-AABB is done.
//
// The advantage: If the OrientedAABB resembles a thin and long object (like an
// edge with a small collision margin), then its AABB in the unrotated state
// will be very tight.
// The disadvantage: The rotation of the edge points is costly and also each
// intersection test requires a calculation of the edges AABB. Its still
// cheaper than a OBB intersection test.
class OrientedAABB
{
public:
    OrientedAABB()
    {
    }

    bool intersects(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2) const
    {
        // rotate p1 and p2 in new space
        Eigen::Vector3d p1Rot = mRotation * (p1 - mTranslation);
        Eigen::Vector3d p2Rot = mRotation * (p2 - mTranslation);

        Eigen::Vector3d min = p1Rot.cwiseMin(p2Rot);
        Eigen::Vector3d max = p1Rot.cwiseMax(p2Rot);

        // perform AABB check
        return -mWidth < max(0) && min(0) < mWidth &&
                0 < max(1) && min(1) < mHeight &&
                -mWidth < max(2) && min(2) < mWidth;
    }

    const Eigen::Quaterniond& getRotation() const
    {
        return mRotation;
    }

    void setRotation(const Eigen::Quaterniond& rotation)
    {
        mRotation = rotation;
    }

    const Eigen::Vector3d& getTranslation() const
    {
        return mTranslation;
    }

    void setTranslation(const Eigen::Vector3d& translation)
    {
        mTranslation = translation;
    }

    double getHeigth() const
    {
        return mHeight;
    }

    void setHeight(double height)
    {
        mHeight = height;
    }

    double getWidth() const
    {
        return mWidth;
    }

    void setWidth(double width)
    {
        mWidth = width;
    }

private:
    Eigen::Quaterniond mRotation;
    Eigen::Vector3d mTranslation;
    double mHeight;
    double mWidth;
};

#endif // ORIENTEDAABB_H
