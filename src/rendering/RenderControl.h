#ifndef RENDERCONTROL_H
#define RENDERCONTROL_H

#include <data_structures/DataStructures.h>
#include <memory>

class Domain;
class GLWidget;
class Renderer;
class RendererProxy;
class RenderThread;

// Manages RenderThread and Renderer.
// Starts the RenderThread.
// Obtain the Renderer or RendererProxy with the corresponding getters.
class RenderControl
{
public:
    RenderControl(GLWidget* glWidget);

    void handlePreRenderingStep();

    Eigen::Vector3f getHeadlightPosition() const;
    void setHeadlightPosition(const Eigen::Vector3f& pos);

    Renderer* getRenderer();
    RendererProxy* getRendererProxy();

private:
    // This domain is used for processing Renderering operations
    // in the QT main thread at the beginning of updateGL().
    std::shared_ptr<Domain> mDomain;

    GLWidget* mGlWidget;

    std::shared_ptr<RenderThread> mRenderThread;

    std::shared_ptr<Renderer> mRenderer;
    std::shared_ptr<RendererProxy> mRendererProxy;
};

#endif // RENDERCONTROL_H
