#include "RenderControl.h"
#include "RenderThread.h"
#include "Renderer.h"

#include <glwidget.h>


RenderControl::RenderControl(GLWidget* glWidget)
    : mGlWidget(glWidget)
{
    mDomain = std::make_shared<Domain>();
    mRenderThread = std::make_shared<RenderThread>(glWidget);

    mRenderer = std::make_shared<Renderer>(mDomain.get());
    mRendererProxy = std::make_shared<RendererProxy>(mRenderer.get());

    mRenderThread->start();

}

void RenderControl::handlePreRenderingStep()
{
    // head light
    // Just use the light direction and global parallel light.
//    QVector3D pos = mGlWidget->getCameraPos();
    QVector3D dir = mGlWidget->getCameraDir();

//    float distance = 1.0f;
//    mRenderer->setLightPosition(
//                Eigen::Vector3f(pos.x(), pos.y(), pos.z()) -
//                distance * Eigen::Vector3f(dir.x(), dir.y(), dir.z()));

    mRenderer->setLightDirection(
                Eigen::Vector3f(dir.x(), dir.y(), dir.z()));

    mRenderer->handlePreRenderingStep();
}

Eigen::Vector3f RenderControl::getHeadlightDirection() const
{
    return mRenderer->getLightDirection();
}

void RenderControl::setHeadlightDirection(const Eigen::Vector3f& dir)
{
    mRenderer->setLightDirection(dir);
}

Renderer* RenderControl::getRenderer()
{
    return mRenderer.get();
}

RendererProxy* RenderControl::getRendererProxy()
{
    return mRendererProxy.get();
}
