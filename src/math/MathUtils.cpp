#include "MathUtils.h"

#include <iostream>

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

Eigen::Vector3d MathUtils::perp(const Eigen::Vector3d& v)
{
    static const double sqrt_inv3 = std::sqrt(double(1) / double(3));

    Eigen::Vector3d u;
    if (std::abs(v(0)) >= sqrt_inv3)
    {
        u(0) = v(1);
        u(1) = -v(0);
        u(2) = 0;
    }
    else
    {
        u(0) = 0;
        u(1) = v(2);
        u(2) = -v(1);
    }

    u.normalize();
    return u;
}

bool MathUtils::projectPointOnTriangle(
        const Eigen::Vector3d& p0,
        const Eigen::Vector3d& p1,
        const Eigen::Vector3d& p2,
        const Eigen::Vector3d& p,
        Eigen::Vector3d& inter,
        Eigen::Vector3d& bary,
        bool& isInside)
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

    // This clamping gives not an exact orthogonal point to the edge.
    isInside = true;
    if (w1 < 0)
    {
        w1 = 0;
        isInside = false;
    }
    else if (w1 > 1)
    {
        w1 = 1;
        isInside = false;
    }

    if (w2 < 0)
    {
        w2 = 0;
        isInside = false;
    }
    else if (w2 > 1)
    {
        w2 = 1;
        isInside = false;
    }

    bary[0] = w1;
    bary[1] = w2;
    bary[2] = 1.0 - w1 - w2;

    if (bary[2] < 0)
    {
        // This gives not an exact orthogonal point to the edge.
        const double w12 = w1 + w2;
        bary[0] -= w2 / (w12)*(w12 - 1);
        bary[1] -= w1 / (w12)*(w12 - 1);
        bary[2] = 0;
        isInside = false;
    }

    inter = p2 + bary[0] * x13 + bary[1] * x23;

    return true;
}

bool MathUtils::projectEdgeOnEdge(
        const Eigen::Vector3d& x1,
        const Eigen::Vector3d& x2,
        const Eigen::Vector3d& x3,
        const Eigen::Vector3d& x4,
        Eigen::Vector3d& inter1,
        Eigen::Vector3d& inter2,
        Eigen::Vector2d& bary,
        bool& isInside)
{
    const Eigen::Vector3d x21 = x2 - x1;
    const Eigen::Vector3d x43 = x4 - x3;
    const Eigen::Vector3d x31 = x3 - x1;

    // compute inv matrix a,b,b,c
    double a = x21.dot(x21);
    double b = -x21.dot(x43);
    double c = x43.dot(x43);
    const double det = a*c - b*b;
    if (fabs(det) < 1.0e-9)
        return false;

    double d1 = x21.dot(x31);
    double d2 = -x43.dot(x31);

    double w1 = (c*d1 - b*d2) / det;
    double w2 = (a*d2 - b*d1) / det;

    // This clamping gives not an exact orthogonal point to the edge.
    isInside = true;
    if (w1 < 0)
    {
        w1 = 0;
        isInside = false;
        return true;
    }
    else if (w1 > 1)
    {
        w1 = 1;
        isInside = false;
        return true;
    }

    if (w2 < 0)
    {
        w2 = 0;
        isInside = false;
        return true;
    }
    else if (w2 > 1)
    {
        w2 = 1;
        isInside = false;
        return true;
    }

    bary[0] = w1;
    bary[1] = w2;

//    std::cout << "x1 = " << x1.transpose() << "\n"
//              << "x2 = " << x2.transpose() << "\n"
//              << "x21 = " << x21.transpose() << "\n"
//              << "x43 = " << x43.transpose() << "\n";

    inter1 = x1 + w1 * x21;
    inter2 = x3 + w2 * x43;

    return true;
}
