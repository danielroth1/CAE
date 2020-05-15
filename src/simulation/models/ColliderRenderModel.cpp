#include "ColliderRenderModel.h"

#include <rendering/object/RenderLine.h>
#include <rendering/object/RenderLines.h>
#include <rendering/object/RenderPoints.h>

#include <rendering/RenderMaterial.h>
#include <rendering/Renderer.h>

#include <simulation/collision_detection/CollisionManager.h>


ColliderRenderModel::ColliderRenderModel(
        const std::shared_ptr<CollisionManager>& collisionManager,
        bool renderLines,
        bool renderPoints)
    : mCollisionManager(collisionManager)
    , mRenderLines(renderLines)
    , mRenderPoints(renderPoints)
{
    mLineLength = 0.2f;

    if (renderLines)
    {
        // grass-green color
        mRenderLinesCollisions = std::make_shared<RenderLines>();
        mRenderLinesCollisions->setRenderMaterial(
                    RenderMaterial::createFromColor({0.71f, 1.0f, 0.0f, 1.0}));

        // light blue
        mRenderLinesContacts = std::make_shared<RenderLines>();
        mRenderLinesContacts->setRenderMaterial(
                    RenderMaterial::createFromColor({0.0f, 0.58f, 1.0f, 1.0}));
    }

    if (renderPoints)
    {
        // red
        mRenderPointsCollisions = std::make_shared<RenderPoints>();
        mRenderPointsCollisions->setRenderMaterial(
                    RenderMaterial::createFromColor({1.0f, 0.0f, 0.0f, 1.0}));

        // violet
        mRenderPointsContacts = std::make_shared<RenderPoints>();
        mRenderPointsContacts->setRenderMaterial(
                    RenderMaterial::createFromColor({0.7f, 0.0f, 1.0f, 1.0}));
    }
}

void ColliderRenderModel::reset()
{

}

void ColliderRenderModel::update()
{
    updateRenderCollisions(mRenderLinesContacts,
                           mRenderPointsContacts,
                           0,
                           mCollisionManager->getNumContacts());

    updateRenderCollisions(mRenderLinesCollisions,
                           mRenderPointsCollisions,
                           mCollisionManager->getNumContacts(),
                           mCollisionManager->getCollisions().size());
}

void ColliderRenderModel::revalidate()
{

}

void ColliderRenderModel::accept(RenderModelVisitor& /*v*/)
{

}

void ColliderRenderModel::addToRenderer(Renderer* renderer)
{
    if (mRenderLines)
    {
        renderer->addRenderObject(mRenderLinesCollisions);
        renderer->addRenderObject(mRenderLinesContacts);
    }

    if (mRenderPoints)
    {
        renderer->addRenderObject(mRenderPointsCollisions);
        renderer->addRenderObject(mRenderPointsContacts);
    }

}

void ColliderRenderModel::removeFromRenderer(Renderer* renderer)
{
    if (mRenderLines)
    {
        renderer->removeRenderObject(mRenderLinesCollisions);
        renderer->removeRenderObject(mRenderLinesContacts);
    }

    if (mRenderPoints)
    {
        renderer->removeRenderObject(mRenderPointsCollisions);
        renderer->removeRenderObject(mRenderPointsContacts);
    }
}

void ColliderRenderModel::setVisible(bool visible)
{
    if (mRenderLines)
    {
        mRenderLinesCollisions->setVisible(visible);
        mRenderLinesContacts->setVisible(visible);
    }

    if (mRenderPoints)
    {
        mRenderPointsCollisions->setVisible(visible);
        mRenderPointsContacts->setVisible(visible);
    }

    RenderModel::setVisible(visible);
}

void ColliderRenderModel::updateRenderCollisions(
        const std::shared_ptr<RenderLines>& renderLines,
        const std::shared_ptr<RenderPoints>& renderPoints,
        size_t startIndex,
        size_t endIndex)
{
    size_t size = endIndex - startIndex;
    bool needsUpdate = false;

    // lines
    if (mRenderLines)
    {
        auto lines = renderLines->getLines().lock();
        size_t sizeBefore = lines->size();

        lines->resize(size * 2);

        for (size_t i = startIndex; i < endIndex; ++i)
        {
            const Collision& c = mCollisionManager->getCollisions()[i].getCollision();
            Eigen::Vectorf mid = 0.5 * (c.getPointB() - c.getPointA()).cast<float>();
            (*lines)[(i - startIndex) * 2] = c.getPointA().cast<float>() + mid + mLineLength * c.getNormal().cast<float>();
            (*lines)[(i - startIndex) * 2 + 1] = c.getPointA().cast<float>() + mid;// - mLineLength * c.getNormal().cast<float>();
        }

        needsUpdate = sizeBefore != lines->size();
    }

    // points
    if (mRenderPoints)
    {
        auto points = renderPoints->getPoints().lock();

        size_t sizeBefore = points->size();

        points->resize(size);

        for (size_t i = startIndex; i < endIndex; ++i)
        {
            const Collision& c = mCollisionManager->getCollisions()[i].getCollision();
            Eigen::Vectorf mid = 0.5 * (c.getPointB() - c.getPointA()).cast<float>();
            (*points)[i - startIndex] = c.getPointA().cast<float>() + mid;
        }

        needsUpdate = sizeBefore != points->size();
    }

    // only update of RenderLines required if the size of points changed.
    if (needsUpdate)
    {
        if (mRenderLines)
        {
            renderLines->update();
        }

        if (mRenderPoints)
        {
            renderPoints->update();
        }
    }
}
