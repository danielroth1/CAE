#include "RenderControl.h"
#include "RenderThread.h"
#include "Renderer.h"


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
    mRenderer->handlePreRenderingStep();
}

Renderer* RenderControl::getRenderer()
{
    return mRenderer.get();
}

RendererProxy* RenderControl::getRendererProxy()
{
    return mRendererProxy.get();
}
