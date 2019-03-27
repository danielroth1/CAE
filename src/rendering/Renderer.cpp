#include "Renderer.h"
#include "data_structures/DataStructures.h"
#include <rendering/object/RenderLine.h>
#include <rendering/object/RenderObjectVisitor.h>
#include <rendering/object/RenderPoints.h>
#include <rendering/object/RenderPolygon2D.h>
#include <rendering/object/RenderScreenRectangle.h>
#include "LightRenderer.h"
#include "RenderObjectFactory.h"
#include <GL/glew.h>
#include <glwidget.h>
#include <iostream>
#include <times/timing.h>

using namespace Eigen;


Renderer::Renderer(Domain* domain)
    : mDomain(domain)
{
    qRegisterMetaType<std::shared_ptr<RenderObject>>();

    mFactory = std::make_shared<RenderObjectFactory>(domain);
//    QObject::connect(this, SIGNAL(addRenderOjectSignal(std::shared_ptr<RenderObject>)),
//            this, SLOT(addRenderOjectSlot(std::shared_ptr<RenderObject>)));

//    QObject::connect(this, SIGNAL(removeRenderObjectSignal(std::shared_ptr<RenderObject>)),
//            this, SLOT(removeRenderObjectSlot(std::shared_ptr<RenderObject>)));

}

Domain* Renderer::getDomain()
{
    return mDomain;
}

void Renderer::draw()
{
    START_TIMING("Renderer::draw()")
    if (mEnableWireframe)
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    else
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    if (mLightRenderer)
        mLightRenderer->drawLight();
    glEnable(GL_LIGHTING);

    class Visitor : public RenderObjectVisitor
    {
    public:
        Visitor(Renderer& renderer)
            : mR(renderer)
        {
        }
        void visit(RenderPolygon2D& /*rp*/)
        {
            // calculate normals?
//            rp.refreshBuffers();
//            rp.drawArray();
//            rp.draw();
//            rp.drawImmediate();
        }

        void visit(RenderPoints& /*rp*/)
        {
//            rp.drawImmediate();
            // TODO: implement this
        }

        void visit(RenderScreenRectangle& /*rsr*/)
        {
//            rsr.drawImmediate();
        }

        void visit(RenderLine& /*model*/)
        {
//            model.draw();
        }

        Renderer& mR;
    };
    Visitor v(*this);

    for (const std::shared_ptr<RenderObject>& ro : mRenderObjects)
    {
        if (ro->isVisible())
        {
            // refresh buffers needs to be called for world space polygons
            // whichs data changed
            START_TIMING("RenderObject::refreshBuffers()")
            ro->refreshBuffers();
            STOP_TIMING
            ro->accept(v);
//            START_TIMING("RenderObject::refreshBuffers()")
//            ro->refreshBuffers();
//            STOP_TIMING
            START_TIMING("RenderObject::draw()")
            ro->draw();
            STOP_TIMING
        }
    }

//    glColor3f(0.0f, 1.0f, 0.0f);
//    // Linear forces
//    for (auto lf_it = m_simulation->getLinearForces().begin();
//         lf_it != m_simulation->getLinearForces().end();
//         ++lf_it)
//    {
//        LinearForce* lf = *lf_it;
//        Vector source = m_simulation->getVertex(lf->getSourceVertexId());
//        Vector target = lf->getTargetVertex();
//        glBegin(GL_LINES);
//            glVertex3f(source(0), source(1), source(2));
//            glVertex3f(target(0), target(1), target(2));
//        glEnd();
//        // TODO: fix debugger
//    }
    STOP_TIMING
}

void Renderer::initialize()
{
    for (const std::shared_ptr<RenderObject>& ro : mRenderObjects)
    {
        ro->initialize();
    }
    mProxy = std::make_shared<RendererProxy>(this);
    mLightRenderer = std::make_shared<LightRenderer>();
    mLightRenderer->initialize();

    // enable transparency
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable( GL_BLEND );
}

void Renderer::enableWireframe(bool enable)
{
    mEnableWireframe = enable;
}

void Renderer::handlePreRenderingStep()
{
    mDomain->processOperations();
}

void Renderer::addRenderObject(std::shared_ptr<RenderObject> ro)
{
//    std::cout << "SIGNAL: add to renderer\n";
    mProxy->addRenderObjectSlot(ro);
}

void Renderer::removeRenderObject(std::shared_ptr<RenderObject> ro)
{
    mProxy->removeRenderObjectSlot(ro);
}

std::shared_ptr<RenderObjectFactory>& Renderer::getRenderObjectFactory()
{
    return mFactory;
}

void Renderer::addRenderObjectSlot(std::shared_ptr<RenderObject> ro)
{
//    mGLWidget->makeCurrent();
//    std::cout << "SLOT: add to renderer\n";

    START_TIMING("Renderer::addRenderObjectSlot");
    auto it = std::find(mRenderObjects.begin(), mRenderObjects.end(), ro);
    if (it == mRenderObjects.end())
    {
        mRenderObjects.push_back(ro);
    }
    ro->createBuffers();
    STOP_TIMING;

//    printInfo();
}

void Renderer::removeRenderObjectSlot(std::shared_ptr<RenderObject> ro)
{
//    mGLWidget->makeCurrent();
    // TODO: is the RenderObject really removed?
    auto it = std::find(mRenderObjects.begin(), mRenderObjects.end(), ro);
    if (it != mRenderObjects.end())
    {
        mRenderObjects.erase(it);
        std::cout << "size = " << mRenderObjects.size() << std::endl;
    }
}

void Renderer::printInfo()
{
    class Visitor : public RenderObjectVisitor
    {
    public:
        Visitor(Renderer& renderer)
            : mR(renderer)
        {
            nTriangles = 0;
            nVertices = 0;
        }
        void visit(RenderPolygon2D& rp)
        {
            nTriangles += rp.getNumberOfTriangles();
            nVertices += rp.getNumberOfPositions();
        }

        void visit(RenderPoints& /*rp*/)
        {
        }

        void visit(RenderScreenRectangle& /*rsr*/)
        {
        }

        void visit(RenderLine& /*model*/)
        {
        }

        Renderer& mR;
        size_t nTriangles;
        size_t nVertices;
    };
    Visitor v(*this);

    for (const std::shared_ptr<RenderObject>& ro : mRenderObjects)
    {
        if (ro->isVisible())
        {
            ro->accept(v);
        }
    }

    std::cout << "#Vertices: " << v.nVertices << ", #Triangles: " << v.nTriangles << std::endl;
}

//RendererProxy::RendererProxy(Renderer* renderer)
//    : mRenderer(renderer)
//{
//    connect(this, SIGNAL(addRenderOjectSignal(std::shared_ptr<RenderObject>)),
//            this, SLOT(addRenderOjectSlot(std::shared_ptr<RenderObject>)));

//    connect(this, SIGNAL(removeRenderObjectSignal(const std::shared_ptr<RenderObject>&)),
//            this, SLOT(removeRenderObjectSlot(const std::shared_ptr<RenderObject>&)));

//}

//void RendererProxy::addRenderOject(std::shared_ptr<RenderObject> ro)
//{
//    emit addRenderOjectSignal(ro);
//}

//void RendererProxy::removeRenderObject(const std::shared_ptr<RenderObject>& ro)
//{
//    emit removeRenderObjectSignal(ro);
//}

//void RendererProxy::addRenderOjectSlot(std::shared_ptr<RenderObject> ro)
//{
//    mRenderer->addRenderObject(ro);
//}

//void RendererProxy::removeRenderObjectSlot(const std::shared_ptr<RenderObject>& ro)
//{
//    mRenderer->removeRenderObject(ro);
//}
