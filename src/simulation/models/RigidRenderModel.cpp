#include "RigidRenderModel.h"

#include <rendering/object/RenderLine.h>

#include <scene/data/geometric/Polygon3D.h>

#include <simulation/rigid/RigidBody.h>

#include <rendering/Renderer.h>


RigidRenderModel::RigidRenderModel(
        std::shared_ptr<Polygon> polygon,
        RigidBody* rb)
    : PolygonRenderModel(polygon, false)
    , mRigid(rb)
{
    mRenderLine = std::make_shared<RenderLine>();
}

void RigidRenderModel::reset()
{

    PolygonRenderModel::reset();
}

void RigidRenderModel::update()
{
    {
        auto line = mRenderLine->getLine().lock();
        line->clear();

        for (size_t i = 0; i < mPolygon->getSize(); ++i)
        {
            line->push_back(mRigid->getPosition().cast<float>());
            line->push_back((mRigid->getOrientation() * mPolygon->getPositionBS(i)).cast<float>());
        }
    }

    PolygonRenderModel::update();
}

void RigidRenderModel::addToRenderer(Renderer* renderer)
{
    renderer->addRenderObject(mRenderLine);
    PolygonRenderModel::addToRenderer(renderer);
}

void RigidRenderModel::removeFromRenderer(Renderer* renderer)
{
    renderer->removeRenderObject(mRenderLine);
    PolygonRenderModel::removeFromRenderer(renderer);
}
