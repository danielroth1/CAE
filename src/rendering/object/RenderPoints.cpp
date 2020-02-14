#include "RenderObjectVisitor.h"
#include "RenderPoints.h"

#include <iostream>

#include <rendering/RenderMaterial.h>

using namespace Eigen;

RenderPoints::RenderPoints()
    : RenderObject()
{

}

void RenderPoints::accept(RenderObjectVisitor& visitor)
{
    visitor.visit(*this);
}

void RenderPoints::draw()
{
    drawImmediate();
}

void RenderPoints::drawImmediate()
{
    glDisable(GL_LIGHTING);
    setPolygonMode();
    mRenderMaterial->glColorAmbient();

    // render selected vertices
    glPushMatrix();
    {
        auto pointsLock = mPoints.lock();
        for (const Vectorf& v : *pointsLock)
        {
            //glLoadIdentity();
            glPushMatrix();
                glTranslatef(v(0), v(1), v(2));
                GLdouble modelview[16];
                glGetDoublev( GL_MODELVIEW_MATRIX, modelview );
                double x, y, z;
                x = modelview[12];
                y = modelview[13];
                z = modelview[14];
                glLoadIdentity();
                glTranslatef(static_cast<float>(x),
                             static_cast<float>(y),
                             static_cast<float>(z));
                //glScalef( 10, 10, 10 );

                //glutWireCube( 0.1 );
                //std::cout << "draw at " << v(0) << ", " << v(1) << ", " << v(2) << "\n";
                glScalef(.02f, .02f, .02f);
                glBegin( GL_QUADS );
                    glVertex2f( -1.0f, -1.0f );
                    glVertex2f(  1.0f, -1.0f );
                    glVertex2f(  1.0f,  1.0f );
                    glVertex2f( -1.0f,  1.0f );
                glEnd();
            glPopMatrix();
        }
    }
    glPopMatrix();

    glEnable(GL_LIGHTING);
}

void RenderPoints::drawArray()
{
    setPolygonMode();
    mRenderMaterial->glMaterial();
    // Not implemented
}

void RenderPoints::drawVBO()
{
    setPolygonMode();
    mRenderMaterial->glMaterial();
    // Not implemented
}

void RenderPoints::update()
{
    // Not implemented
}

void RenderPoints::createBuffers()
{
    // Not implemented
}

void RenderPoints::refreshBuffers()
{
    // Not implemented
}

void RenderPoints::cleanup()
{
    // Not implemented
}

void RenderPoints::initialize()
{
    // Not implemented
}

Monitor<Vectorfs>& RenderPoints::getPoints()
{
    return mPoints;
}
