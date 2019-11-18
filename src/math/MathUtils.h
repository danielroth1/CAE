#ifndef MATHUTILS_H
#define MATHUTILS_H

#include <Eigen/Core>
#include <Eigen/Geometry>

// Provides various usefull matematical operations.
class MathUtils
{
public:
    MathUtils();

    // Note: not implemented
    // Quaternion rotation
    // Rotates vector v by quaternion q and returns the resulting vector.
    // \vec{p} = \frac{1}{2}\unQuat(\vec{q} \otimes \quat(\vec{x}) \otimes \vec{q}^{-1})
    static Eigen::Vector3d rotate(
            const Eigen::Vector3d& v,
            const Eigen::Quaterniond& q);

    // Note: not implemented
    // Rotates vector v by quaternion q and writes the result to
    // the given result reference.
    // \vec{p} = \frac{1}{2}\unQuat(\vec{q} \otimes \quat(\vec{x}) \otimes \vec{q}^{-1})
    static void rotate(
            const Eigen::Vector3d& v,
            const Eigen::Quaterniond& q,
            Eigen::Vector3d& result);

    // Note: not implemented
    // Rotates matrix m by quaternion q and returns the resulting matrix.
    static Eigen::Matrix3d rotate(
            const Eigen::Matrix3d& m,
            const Eigen::Quaterniond& q);

    // Note: not implemented
    // Rotates matrix m by quaternion q and writes the result to
    // the given result reference.
    static Eigen::Matrix3d rotate(
            const Eigen::Matrix3d& m,
            const Eigen::Quaterniond& q,
            Eigen::Matrix3d& result);

    // Projects p on the triangle (p0, p1, p2) and returns the projected point
    // "inter" and baryzentric coordinates "bary". Returns false if the problem
    // has no solution because it was illconditioned (triangle is an edge
    // because two points are identical.)
    // \param isInside - is set true if the projection is on top of the triangle
    //      and clamping wasn't necessary
    static bool projectPointOnTriangle(
            const Eigen::Vector3d& p0,
            const Eigen::Vector3d& p1,
            const Eigen::Vector3d& p2,
            const Eigen::Vector3d& p,
            Eigen::Vector3d& inter,
            Eigen::Vector3d& bary,
            bool& isInside);

    static bool projectEdgeOnEdge(
            const Eigen::Vector3d& p0,
            const Eigen::Vector3d& p1,
            const Eigen::Vector3d& p2,
            const Eigen::Vector3d& p3,
            Eigen::Vector3d& inter1,
            Eigen::Vector3d& inter2,
            Eigen::Vector2d& bary,
            bool& isInside);
};

#endif // MATHUTILS_H
