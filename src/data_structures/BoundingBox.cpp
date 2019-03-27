#include "BoundingBox.h"

#include <Eigen/Dense>
#include <memory>

BoundingBox::BoundingBox()
{

}

Vector& BoundingBox::min()
{
    return m_min;
}

Vector& BoundingBox::max()
{
    return m_max;
}

Vector& BoundingBox::mid()
{
    return m_mid;
}

Vector& BoundingBox::size()
{
    return m_size;
}
