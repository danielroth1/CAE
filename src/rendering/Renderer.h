#ifndef RENDERER_H
#define RENDERER_H

// Includes
#include <QObject>
#include <memory>
#include <proxy/ProxyDefs.h>
#include <vector>

// Forward Declarations
class GLWidget;
class LightRenderer;
class RendererProxy;
class RenderObject;
class RenderObjectFactory;

Q_DECLARE_METATYPE(std::shared_ptr<RenderObject>)

class Renderer : public QObject, public std::enable_shared_from_this<Renderer>
{
    Q_OBJECT

public:
    Renderer(Domain* domain);

    Domain* getDomain();

    void draw();

    void initialize();

    // This function is called with an active GL context.
    // Processes all operations that require the GL context.
    void handlePreRenderingStep();

    // Adds render object and creates its buffers.
    // This method is completely thread safe.
    void addRenderObject(std::shared_ptr<RenderObject> ro);

    // This method is completely thread safe.
    void removeRenderObject(std::shared_ptr<RenderObject> ro);

    Eigen::Vector3f getLightDirection() const;
    void setLightDirection(const Eigen::Vector3f& dir);

    void setLightPosition(const Eigen::Vector3f& pos);

    std::shared_ptr<RenderObjectFactory>& getRenderObjectFactory();

    // Prints info about currently number of rendered triangles/ vertices
    void printInfo();

//public slots:
    void addRenderObjectSlot(std::shared_ptr<RenderObject> ro);
    void removeRenderObjectSlot(std::shared_ptr<RenderObject> ro);
    void setLightDirectionSlot(Eigen::Vector3f dir);
    void setLightPositionSlot(Eigen::Vector3f pos);

private:

    Domain* mDomain;

    GLWidget* mGLWidget;

    std::vector<std::shared_ptr<RenderObject>> mRenderObjects;

    std::shared_ptr<RendererProxy> mProxy;

    std::shared_ptr<LightRenderer> mLightRenderer;

    std::shared_ptr<RenderObjectFactory> mFactory;

};

PROXY_CLASS(RendererProxy, Renderer, mR,
            PROXY_FUNCTION(Renderer, mR, addRenderObjectSlot,
                           PL(std::shared_ptr<RenderObject> ro),
                           PL(ro))
            PROXY_FUNCTION(Renderer, mR, removeRenderObjectSlot,
                           PL(std::shared_ptr<RenderObject> ro),
                           PL(ro))
            PROXY_FUNCTION(Renderer, mR, setLightDirectionSlot,
                           PL(Eigen::Vector3f dir),
                           PL(dir))
            PROXY_FUNCTION(Renderer, mR, setLightPositionSlot,
                           PL(Eigen::Vector3f pos),
                           PL(pos))
            )

#endif // RENDERER_H
