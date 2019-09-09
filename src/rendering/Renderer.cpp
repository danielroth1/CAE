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
    START_TIMING_RENDERING("Renderer::draw()")

    if (mLightRenderer)
        mLightRenderer->drawLight();
    glEnable(GL_LIGHTING);

    for (const std::shared_ptr<RenderObject>& ro : mRenderObjects)
    {
        if (ro->isVisible())
        {
            // refresh buffers needs to be called for world space polygons
            // whichs data changed
            START_TIMING_RENDERING("RenderObject::refreshBuffers()")
            ro->refreshBuffers();
            STOP_TIMING_RENDERING

            START_TIMING_RENDERING("RenderObject::draw()")
            ro->draw();
            STOP_TIMING_RENDERING
        }
    }

    STOP_TIMING_RENDERING
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

Vector3f Renderer::getLightDirection() const
{
    return mLightRenderer->getLightDirection();
}

void Renderer::setLightDirection(const Vector3f& pos)
{
    mProxy->setLightDirectionSlot(pos);
}

std::shared_ptr<RenderObjectFactory>& Renderer::getRenderObjectFactory()
{
    return mFactory;
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

void Renderer::addRenderObjectSlot(std::shared_ptr<RenderObject> ro)
{
//    mGLWidget->makeCurrent();
//    std::cout << "SLOT: add to renderer\n";

    START_TIMING_RENDERING("Renderer::addRenderObjectSlot");
    auto it = std::find(mRenderObjects.begin(), mRenderObjects.end(), ro);
    if (it == mRenderObjects.end())
    {
        mRenderObjects.push_back(ro);
        ro->createBuffers();
    }
    STOP_TIMING_RENDERING;

//    printInfo();
}

void Renderer::removeRenderObjectSlot(std::shared_ptr<RenderObject> ro)
{
//    mGLWidget->makeCurrent();
    auto it = std::find(mRenderObjects.begin(), mRenderObjects.end(), ro);
    if (it != mRenderObjects.end())
    {
        mRenderObjects.erase(it);
        std::cout << "size = " << mRenderObjects.size() << std::endl;
    }
}

void Renderer::setLightDirectionSlot(Eigen::Vector3f pos)
{
    mLightRenderer->setLightDirection(pos);
}
