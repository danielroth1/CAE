#ifndef ORIENTEDBOUNDINGBOX_H
#define ORIENTEDBOUNDINGBOX_H

#include <Eigen/Dense>
#include <array>

// A oriented bounding box (OBB) is a bounding volume that is represented
// by a box which can be rotated.
// * A OBB approximates most geometries better than an axis-aligned bounding box
// (AABB) but its intersection test is more expensive.
// * Finding an optimal OBB is more complicated and usually not feasible while
// simulating. Instead, OBBs should be calculated in a precomputation step.
// This limits their usage to structures like rigid bodies where the rigids
// rotation and translation can be used to offset the OBBs rotation and translation.
//
class OrientedBoundingBox
{
public:
    OrientedBoundingBox();

    bool intersects(const OrientedBoundingBox& b) const;

    Eigen::Vector3d& getPosition()
    {
        return mPosition;
    }

    const Eigen::Vector3d& getPosition() const
    {
        return mPosition;
    }

    Eigen::Vector3d& getHalfSizes()
    {
        return mHalfSizes;
    }

    const Eigen::Vector3d& getHalfSizes() const
    {
        return mHalfSizes;
    }

    std::array<Eigen::Vector3d, 3>& getAxes()
    {
        return mAxes;
    }

    const std::array<Eigen::Vector3d, 3>& getAxes() const
    {
        return mAxes;
    }

private:
    Eigen::Vector3d mPosition;
    Eigen::Vector3d mHalfSizes;
    std::array<Eigen::Vector3d, 3> mAxes;
};

#endif // ORIENTEDBOUNDINGBOX_H
