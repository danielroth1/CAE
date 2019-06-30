#include "MeshInterpolator.h"
#include "Polygon.h"
#include "Polygon2DAccessor.h"

#include <algorithm>
#include <iostream>


MeshInterpolator::MeshInterpolator(
        const std::shared_ptr<Polygon>& source,
        const std::shared_ptr<Polygon>& target)
    : mSource(source)
    , mTarget(target)
{
    mSourceAccessor = source->createAccessor();
    mTargetAccessor = target->createAccessor();
}

std::shared_ptr<Polygon> MeshInterpolator::getSource() const
{
    return mSource;
}

std::shared_ptr<Polygon> MeshInterpolator::getTarget() const
{
    return mTarget;
}

std::shared_ptr<Polygon2DAccessor> MeshInterpolator::getSource2DAccessor() const
{
    return mSourceAccessor;
}

std::shared_ptr<Polygon2DAccessor> MeshInterpolator::getTarget2DAccessor() const
{
    return mTargetAccessor;
}

MeshInterpolator::~MeshInterpolator()
{

}
