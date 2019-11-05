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

bool MathUtils::projectPointOnTriangle(
        const Eigen::Vector3d& p0,
        const Eigen::Vector3d& p1,
        const Eigen::Vector3d& p2,
        const Eigen::Vector3d& p,
        Eigen::Vector3d& inter,
        Eigen::Vector3d &bary)
{
    // see Bridson: Robust treatment of collisions contact and friction for cloth animation
    const Eigen::Vector3d x43 = p - p2;
    const Eigen::Vector3d x13 = p0 - p2;
    const Eigen::Vector3d x23 = p1 - p2;

    // compute inv matrix a,b,b,c
    double a = x13.dot(x13);
    double b = x13.dot(x23);
    double c = x23.dot(x23);
    const double det = a*c - b*b;
    if (fabs(det) < 1.0e-9)
        return false;

    double d1 = x13.dot(x43);
    double d2 = x23.dot(x43);

    double w1 = (c*d1 - b*d2) / det;
    double w2 = (a*d2 - b*d1) / det;

    // this clamping gives not an exact orthogonal point to the edge!!
    if (w1 < 0) w1 = 0;
    if (w1 > 1) w1 = 1;
    if (w2 < 0) w2 = 0;
    if (w2 > 1) w2 = 1;

    bary[0] = w1;
    bary[1] = w2;
    bary[2] = 1.0 - w1 - w2;

    if (bary[2] < 0)
    {
        // this gives not an exact orthogonal point to the edge!!
        const double w12 = w1 + w2;
        bary[0] -= w2 / (w12)*(w12 - 1);
        bary[1] -= w1 / (w12)*(w12 - 1);
        bary[2] = 0;
    }

    inter = p2 + bary[0] * x13 + bary[1] * x23;

    return true;
}
