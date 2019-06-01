#include "RenderLine.h"
#include "RenderObjectVisitor.h"

RenderLine::RenderLine()
    : RenderObject()
{

}

void RenderLine::accept(RenderObjectVisitor& visitor)
{
    visitor.visit(*this);
}

void RenderLine::draw()
{
    drawImmediate();
}

void RenderLine::drawImmediate()
{
    mRenderMaterial.glMaterial();
    glBegin(GL_LINES);
    {
        auto lineLock = mLine.lock();
        for (const Eigen::Vectorf& p : *lineLock)
        {
            glVertex3f(p(0), p(1), p(2));
        }
    }
    glEnd();
}

void RenderLine::drawArray()
{
    // Not implemented
}

void RenderLine::drawVBO()
{
    // Not implemented
}

void RenderLine::update()
{
    // Not implemented
}

void RenderLine::createBuffers()
{
    // Not implemented
}

void RenderLine::refreshBuffers()
{
    // Not implemented
}

void RenderLine::cleanup()
{
    // Not implemented
}

void RenderLine::initialize()
{
    // Not implemented
}

Monitor<std::vector<Eigen::Vectorf>>& RenderLine::getLine()
{
    return mLine;
}
