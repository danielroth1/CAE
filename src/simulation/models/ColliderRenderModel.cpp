#include "ColliderRenderModel.h"

#include <simulation/collision_detection/narrow/Collider.h>

#include <rendering/object/RenderLine.h>
#include <rendering/object/RenderLines.h>
#include <rendering/object/RenderPoints.h>

#include <rendering/Renderer.h>


ColliderRenderModel::ColliderRenderModel(std::shared_ptr<Collider> collider)
    : mCollider(collider)
{
    mLineLength = 0.2f;
    mRenderLines = std::make_shared<RenderLines>();
    mRenderLines->setColor(Eigen::Vector4f(0.71f, 1.0f, 0.0f, 1.0)); // grass-green

    mRenderPoints = std::make_shared<RenderPoints>();
    mRenderPoints->setColor(Eigen::Vector4f(1.0f, 0.0f, 0.0f, 1.0));
}

void ColliderRenderModel::reset()
{

}

void ColliderRenderModel::update()
{
    bool needsUpdate = false;
    {
        auto lines = mRenderLines->getLines().lock();
        size_t sizeBefore = lines->size();

        lines->resize(mCollider->getCollisions().size() * 2);

        for (size_t i = 0; i < mCollider->getCollisions().size(); ++i)
        {
            const Collision& c = mCollider->getCollisions()[i];
            Eigen::Vectorf mid = 0.5 * (c.getPointB() - c.getPointA()).cast<float>();
            (*lines)[i * 2] = c.getPointA().cast<float>() + mid + mLineLength * c.getNormal().cast<float>();
            (*lines)[i * 2 + 1] = c.getPointA().cast<float>() + mid;// - mLineLength * c.getNormal().cast<float>();
        }

        needsUpdate = sizeBefore != lines->size();
    }

    // points
    {
        auto points = mRenderPoints->getPoints().lock();

        size_t sizeBefore = points->size();

        points->resize(mCollider->getCollisions().size());

        for (size_t i = 0; i < mCollider->getCollisions().size(); ++i)
        {
            const Collision& c = mCollider->getCollisions()[i];
            Eigen::Vectorf mid = 0.5 * (c.getPointB() - c.getPointA()).cast<float>();
            (*points)[i] = c.getPointA().cast<float>() + mid;
        }

        needsUpdate = sizeBefore != points->size();
    }

    // only update of RenderLines required if the size of points changed.
    if (needsUpdate)
    {
        mRenderLines->update();
        mRenderPoints->update();
    }
}

void ColliderRenderModel::revalidate()
{

}

void ColliderRenderModel::accept(RenderModelVisitor& /*v*/)
{

}

void ColliderRenderModel::addToRenderer(Renderer* renderer)
{
    renderer->addRenderObject(mRenderLines);
    renderer->addRenderObject(mRenderPoints);
}

void ColliderRenderModel::removeFromRenderer(Renderer* renderer)
{
    renderer->removeRenderObject(mRenderLines);
    renderer->removeRenderObject(mRenderPoints);
}

void ColliderRenderModel::setVisible(bool visible)
{
    mRenderLines->setVisible(visible);
    mRenderPoints->setVisible(visible);
    RenderModel::setVisible(visible);
}
