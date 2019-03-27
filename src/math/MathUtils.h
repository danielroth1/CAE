#ifndef MATHUTILS_H
#define MATHUTILS_H

#include <Eigen/Core>
#include <Eigen/Geometry>

// Provides various usefull matematical operations.
class MathUtils
{
public:
    MathUtils();

    // Quaternion rotation
    // Rotates vector v by quaternion q and returns the resulting vector.
    // \vec{p} = \frac{1}{2}\unQuat(\vec{q} \otimes \quat(\vec{x}) \otimes \vec{q}^{-1})
    Eigen::Vector3d rotate(
            const Eigen::Vector3d& v,
            const Eigen::Quaterniond& q);

    // Rotates vector v by quaternion q and writes the result to
    // the given result reference.
    // \vec{p} = \frac{1}{2}\unQuat(\vec{q} \otimes \quat(\vec{x}) \otimes \vec{q}^{-1})
    void rotate(
            const Eigen::Vector3d& v,
            const Eigen::Quaterniond& q,
            Eigen::Vector3d& result);

    // Rotates matrix m by quaternion q and returns the resulting matrix.
    Eigen::Matrix3d rotate(
            const Eigen::Matrix3d& m,
            const Eigen::Quaterniond& q);

    // Rotates matrix m by quaternion q and writes the result to
    // the given result reference.
    Eigen::Matrix3d rotate(
            const Eigen::Matrix3d& m,
            const Eigen::Quaterniond& q,
            Eigen::Matrix3d& result);
};

#endif // MATHUTILS_H
