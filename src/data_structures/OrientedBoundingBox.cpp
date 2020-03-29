#include "OrientedBoundingBox.h"

OrientedBoundingBox::OrientedBoundingBox()
{

}

bool OrientedBoundingBox::intersects(const OrientedBoundingBox& b) const
{
    double epsilon = 1e-12;

    // Generate a rotation matrix that transforms from world space to this OBB's coordinate space.
    Eigen::Matrix3d R;
    for(int i = 0; i < 3; ++i)
    {
        for(int j = 0; j < 3; ++j)
        {
            R(i, j) = getAxes()[i].dot(b.getAxes()[j]);
        }
    }

    Eigen::Vector3d t = b.getPosition() - getPosition();
    // Express the translation vector in a's coordinate frame.

    t = Eigen::Vector3d(
                t.dot(getAxes()[0]), t.dot(getAxes()[1]), t.dot(getAxes()[2]));

    Eigen::Matrix3d AbsR;
    for(int i = 0; i < 3; ++i)
    {
        for(int j = 0; j < 3; ++j)
        {
            AbsR(i, j) = std::abs(R(i,j)) + epsilon;
        }
    }

    // Test the three major axes of this OBB.
    for(int i = 0; i < 3; ++i)
    {
        float ra = getHalfSizes()[i];
        float rb = b.getHalfSizes().dot(AbsR.col(i));
        if (std::abs(t[i]) > ra + rb)
            return false;
    }

    // Test the three major axes of the OBB b.
    for(int i = 0; i < 3; ++i)
    {
        float ra = getHalfSizes()[0] * AbsR(0,i) + getHalfSizes()[1] * AbsR(1,i) + getHalfSizes()[2] * AbsR(2,i);
        float rb = b.getHalfSizes()[i];
        if (std::abs(t(0) * R(0, i) + t(1) * R(1, i) + t(2) * R(2, i)) > ra + rb)
            return false;
    }

    // Test the 9 different cross-axes.

    // A.x <cross> B.x
    float ra = getHalfSizes()(1) * AbsR(2, 0) + getHalfSizes()(2) * AbsR(1, 0);
    float rb = b.getHalfSizes()(1) * AbsR(0, 2) + b.getHalfSizes()(2) * AbsR(0, 1);
    if (std::abs(t(2) * R(1, 0) - t(1) * R(2, 0)) > ra + rb)
        return false;

    // A.x < cross> B.y
    ra = getHalfSizes()(1) * AbsR(2, 1) + getHalfSizes()(2) * AbsR(1, 1);
    rb = b.getHalfSizes()(0) * AbsR(0, 2) + b.getHalfSizes()(2) * AbsR(0, 0);
    if (std::abs(t(2) * R(1, 1) - t(1) * R(2, 1)) > ra + rb)
        return false;

    // A.x <cross> B.z
    ra = getHalfSizes()(1) * AbsR(2, 2) + getHalfSizes()(2) * AbsR(1, 2);
    rb = b.getHalfSizes()(0) * AbsR(0, 1) + b.getHalfSizes()(1) * AbsR(0, 0);
    if (std::abs(t(2) * R(1, 2) - t(1) * R(2, 2)) > ra + rb)
        return false;

    // A.y <cross> B.x
    ra = getHalfSizes()(0) * AbsR(2, 0) + getHalfSizes()(2) * AbsR(0, 0);
    rb = b.getHalfSizes()(1) * AbsR(1, 2) + b.getHalfSizes()(2) * AbsR(1, 1);
    if (std::abs(t(0) * R(2, 0) - t(2) * R(0, 0)) > ra + rb)
        return false;

    // A.y <cross> B.y
    ra = getHalfSizes()(0) * AbsR(2, 1) + getHalfSizes()(2) * AbsR(0, 1);
    rb = b.getHalfSizes()(0) * AbsR(1, 2) + b.getHalfSizes()(2) * AbsR(1, 0);
    if (std::abs(t(0) * R(2, 1) - t(2) * R(0, 1)) > ra + rb)
        return false;

    // A.y <cross> B.z
    ra = getHalfSizes()(0) * AbsR(2, 2) + getHalfSizes()(2) * AbsR(0, 2);
    rb = b.getHalfSizes()(0) * AbsR(1, 1) + b.getHalfSizes()(1) * AbsR(1, 0);
    if (std::abs(t(0) * R(2, 2) - t(2) * R(0, 2)) > ra + rb)
        return false;

    // A.z <cross> B.x
    ra = getHalfSizes()(0) * AbsR(1, 0) + getHalfSizes()(1) * AbsR(0, 0);
    rb = b.getHalfSizes()(1) * AbsR(2, 2) + b.getHalfSizes()(2) * AbsR(2, 1);
    if (std::abs(t(1) * R(0, 0) - t(0) * R(1, 0)) > ra + rb)
        return false;

    // A.z <cross> B.y
    ra = getHalfSizes()(0) * AbsR(1, 1) + getHalfSizes()(1) * AbsR(0, 1);
    rb = b.getHalfSizes()(0) * AbsR(2, 2) + b.getHalfSizes()(2) * AbsR(2, 0);
    if (std::abs(t(1) * R(0, 1) - t(0) * R(1, 1)) > ra + rb)
        return false;

    // A.z <cross> B.z
    ra = getHalfSizes()(0) * AbsR(1, 2) + getHalfSizes()(1) * AbsR(0, 2);
    rb = b.getHalfSizes()(0) * AbsR(2, 1) + b.getHalfSizes()(1) * AbsR(2, 0);
    if (std::abs(t(1) * R(0, 2) - t(0) * R(1, 2)) > ra + rb)
        return false;

    // No separating axis exists, so the two OBB don't intersect.
    return true;
}
