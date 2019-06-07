#include "RenderObjectVisitor.h"
#include "RenderScreenRectangle.h"
#include <rendering/RenderMaterial.h>
#include <rendering/ViewFrustum.h>

RenderScreenRectangle::RenderScreenRectangle(
        ViewFrustum& viewFrustum,
        const int& xStart,
        const int& yStart,
        const int& xEnd,
        const int& yEnd)
    : RenderObject()
    , mViewFrustum(viewFrustum)
    , mXStart(xStart)
    , mYStart(yStart)
    , mXEnd(xEnd)
    , mYEnd(yEnd)
{

}

void RenderScreenRectangle::accept(RenderObjectVisitor& visitor)
{
    visitor.visit(*this);
}

void RenderScreenRectangle::draw()
{
    drawImmediate();
}

void RenderScreenRectangle::drawImmediate()
{
    int* VP = mViewFrustum.getViewPort();
    const double w = VP[2];
    const double h = VP[3];
    //const double ar = w / h;

    mRenderMaterial->glMaterial();

    // rander selection
    glPushMatrix();
        glMatrixMode(GL_PROJECTION);
        glPushMatrix();
            glLoadIdentity();
            glOrtho( 0, w, 0, h, 0, 1 );
            glMatrixMode(GL_MODELVIEW);
            glLoadIdentity();
            glDisable(GL_CULL_FACE);
            glClear(GL_DEPTH_BUFFER_BIT);

            glBegin( GL_QUADS );
                glVertex2f( static_cast<float>(mXStart), static_cast<float>(mYStart) );
                glVertex2f( static_cast<float>(mXStart), static_cast<float>(mYEnd) );
                glVertex2f( static_cast<float>(mXEnd),  static_cast<float>(mYEnd) );
                glVertex2f( static_cast<float>(mXEnd),  static_cast<float>(mYStart) );
            glEnd();

            glMatrixMode(GL_PROJECTION);
        glPopMatrix();
        glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
}

void RenderScreenRectangle::drawArray()
{
    mRenderMaterial->glMaterial();
    // Not implemented
}

void RenderScreenRectangle::drawVBO()
{
    mRenderMaterial->glMaterial();
    // Not implemented
}

void RenderScreenRectangle::update()
{
    // Not implemented
}

void RenderScreenRectangle::createBuffers()
{
    // Not implemented
}

void RenderScreenRectangle::refreshBuffers()
{
    // Not implemented
}

void RenderScreenRectangle::cleanup()
{
    // Not implemented
}

void RenderScreenRectangle::initialize()
{
    // Not implemented
}
