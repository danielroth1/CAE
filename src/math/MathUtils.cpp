#include "MathUtils.h"

MathUtils::MathUtils()
{

}

Eigen::Vector3d MathUtils::rotate(
        const Eigen::Vector3d& v,
        const Eigen::Quaterniond& q)
{
    // \vec{p} = \frac{1}{2}\unQuat(\vec{q} \otimes \quat(\vec{x}) \otimes \vec{q}^{-1})

}

void MathUtils::rotate(
        const Eigen::Vector3d& v,
        const Eigen::Quaterniond& q,
        Eigen::Vector3d& result)
{

}

Eigen::Matrix3d MathUtils::rotate(
        const Eigen::Matrix3d& m,
        const Eigen::Quaterniond& q)
{

}

Eigen::Matrix3d MathUtils::rotate(
        const Eigen::Matrix3d& m,
        const Eigen::Quaterniond& q,
        Eigen::Matrix3d& result)
{

}
