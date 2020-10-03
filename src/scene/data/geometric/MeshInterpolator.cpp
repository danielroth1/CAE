#include "MeshInterpolator.h"
#include "AbstractPolygon.h"
#include "Polygon2DAccessor.h"

#include <algorithm>
#include <iostream>


MeshInterpolator::MeshInterpolator(
        const std::shared_ptr<AbstractPolygon>& source,
        const std::shared_ptr<AbstractPolygon>& target)
    : mSource(source)
    , mTarget(target)
{
    mSource->update();
    mTarget->update();
    mSourceAccessor = source->createAccessor();
    mTargetAccessor = target->createAccessor();
}

std::shared_ptr<AbstractPolygon> MeshInterpolator::getSource() const
{
    return mSource;
}

std::shared_ptr<AbstractPolygon> MeshInterpolator::getTarget() const
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

void MeshInterpolator::fixRepresentationType()
{
    if (mTarget->getPositionType() != mSource->getPositionType())
    {
        if (mSource->getPositionType() == BSWSVectors::Type::BODY_SPACE)
        {
            mTarget->changeRepresentationToBS(mSource->getCenter());
        }
        else
        {
            mTarget->changeRepresentationToWS();
        }
        mTarget->update();
    }
}
