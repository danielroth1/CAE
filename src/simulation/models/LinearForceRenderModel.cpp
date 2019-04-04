#include "LinearForceRenderModel.h"

#include <scene/model/RenderModelVisitor.h>

#include <rendering/Renderer.h>

#include <simulation/forces/LinearForce.h>

#include <rendering/object/RenderLine.h>

LinearForceRenderModel::LinearForceRenderModel(std::shared_ptr<LinearForce> linearForce)
    : mLinearForce(linearForce)
{
    mRenderLine = std::make_shared<RenderLine>();
    mRenderLine->setColor(Eigen::Vector4f(1.0f, 0.42f, 0.0f, 1.0f)); // orange
    mAlwaysUpdate = true;
}

void LinearForceRenderModel::reset()
{
    update();
}

void LinearForceRenderModel::update()
{

    // rnder correct bs  point
    auto line = mRenderLine->getLine().lock();
    line->clear();
    Eigen::Vector v1 = mLinearForce->getSourceVector().getPoint();
    Eigen::Vector v2 = mLinearForce->getTargetVector().getPoint();
    // write this model and use it where it should be used
    line->push_back(Eigen::Vector3f(
                        static_cast<float>(v1(0)),
                        static_cast<float>(v1(1)),
                        static_cast<float>(v1(2))));
    line->push_back(Eigen::Vector3f(
                        static_cast<float>(v2(0)),
                        static_cast<float>(v2(1)),
                        static_cast<float>(v2(2))));
}

void LinearForceRenderModel::revalidate()
{

}

void LinearForceRenderModel::accept(RenderModelVisitor& v)
{
    v.visit(*this);
}

void LinearForceRenderModel::addToRenderer(Renderer* renderer)
{
    renderer->addRenderObject(mRenderLine);
}

void LinearForceRenderModel::removeFromRenderer(Renderer* renderer)
{
    renderer->removeRenderObject(mRenderLine);
}

void LinearForceRenderModel::setVisible(bool visible)
{
    mRenderLine->setVisible(visible);
    RenderModel::setVisible(visible);
}
