#include "MeshInterpolatorRenderModel.h"

#include <rendering/object/RenderLines.h>
#include <rendering/object/RenderPoints.h>

#include <rendering/RenderMaterial.h>
#include <rendering/Renderer.h>

#include <scene/data/geometric/MeshInterpolator.h>
#include <scene/data/geometric/Polygon.h>
#include <scene/data/geometric/Polygon2DAccessor.h>


MeshInterpolatorRenderModel::MeshInterpolatorRenderModel(
        std::shared_ptr<MeshInterpolator> interpolator,
        bool renderPointsEnablded,
        bool renderLinesEnablded)
    : mInterpolator(interpolator)
    , mRenderPointsEnabled(renderPointsEnablded)
    , mRenderLinesEnabled(renderLinesEnablded)
{
    mRenderer = nullptr;
    reset();
}

MeshInterpolatorRenderModel::~MeshInterpolatorRenderModel()
{

}

void MeshInterpolatorRenderModel::setRenderPointsEnabled(bool renderPointsEnabled)
{
    mRenderPointsEnabled = renderPointsEnabled;
}

bool MeshInterpolatorRenderModel::isRenderPointsEnabled() const
{
    return mRenderPointsEnabled;
}

void MeshInterpolatorRenderModel::setRenderLinesEnabled(bool renderLinesEnabled)
{
    mRenderLinesEnabled = renderLinesEnabled;
}

bool MeshInterpolatorRenderModel::isRenderLinesEnabled() const
{
    return mRenderLinesEnabled;
}

void MeshInterpolatorRenderModel::reset()
{
    if (mRenderer)
    {
        mRenderer->removeRenderObject(mRenderLines);
        mRenderer->removeRenderObject(mRenderPoints);
    }

    mRenderLines = std::make_shared<RenderLines>();
    mRenderPoints = std::make_shared<RenderPoints>();

    mRenderLines->setRenderMaterial(
                RenderMaterial::createFromColor({1.0f, 0.5f, 0.0f, 1.0f}));

    {
        auto indices = mRenderLines->getIndices().lock();
        indices->resize(mInterpolator->getTarget()->getSize());
        for (size_t i = 0; i < mInterpolator->getTarget()->getSize(); ++i)
        {
            (*indices)[i] = {2 * static_cast<unsigned int>(i),
                             2 * static_cast<unsigned int>(i) + 1};
        }
    }

    update();
}

void MeshInterpolatorRenderModel::update()
{
    std::shared_ptr<Polygon2DAccessor> sourceAccessor =
            mInterpolator->getSource2DAccessor();
    std::shared_ptr<Polygon2DAccessor> targetAccessor =
            mInterpolator->getTarget2DAccessor();

    if (mRenderLinesEnabled)
    {
        auto lines = mRenderLines->getLines().lock();
        lines->resize(2 * targetAccessor->getSize());
        for (size_t i = 0; i < targetAccessor->getSize(); ++i)
        {
            Eigen::Vector3f vSource = mInterpolator->getSourcePosition(i).cast<float>();
            Eigen::Vector3f vTarget = targetAccessor->getPosition(i).cast<float>();

            (*lines)[i * 2] = vSource;
            (*lines)[i * 2 + 1] = vTarget;
        }
    }

    if (mRenderPointsEnabled)
    {
        auto points = mRenderPoints->getPoints().lock();
        points->resize(2 * targetAccessor->getSize());
        for (size_t i = 0; i < targetAccessor->getSize(); ++i)
        {
            Eigen::Vector3f vSource = mInterpolator->getSourcePosition(i).cast<float>();
            Eigen::Vector3f vTarget = targetAccessor->getPosition(i).cast<float>();

            (*points)[i * 2] = vSource;
            (*points)[i * 2 + 1] = vTarget;
        }
    }

}

void MeshInterpolatorRenderModel::revalidate()
{
    update();
}

void MeshInterpolatorRenderModel::accept(RenderModelVisitor& /*v*/)
{

}

void MeshInterpolatorRenderModel::addToRenderer(Renderer* renderer)
{
    mRenderer = renderer;
    if (mRenderLines)
        mRenderer->addRenderObject(mRenderLines);

    if (mRenderPoints)
        mRenderer->addRenderObject(mRenderPoints);
}

void MeshInterpolatorRenderModel::removeFromRenderer(Renderer* renderer)
{
    if (mRenderLines)
        renderer->removeRenderObject(mRenderLines);

    if (mRenderPoints)
        renderer->removeRenderObject(mRenderPoints);
}
