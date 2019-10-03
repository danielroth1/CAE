#include "BoundingBox.h"

#include <Eigen/Dense>
#include <memory>

BoundingBox::BoundingBox()
{
    mMin.setZero();
    mMax.setZero();
    mMid.setZero();
    mSize.setZero();
}

