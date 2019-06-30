#include "RenderLines.h"

#include <rendering/RenderMaterial.h>

using namespace Eigen;

RenderLines::RenderLines()
    : RenderObject()
{
    update();
}

void RenderLines::accept(RenderObjectVisitor& /*visitor*/)
{
    // TODO
}

void RenderLines::draw()
{
    drawArray();
}

void RenderLines::drawImmediate()
{
    mRenderMaterial->glMaterial();
    glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );

    glBegin(GL_LINES);

    {
        auto linesLock = mLines.lock();
        for (size_t i = 0; i < linesLock->size() / 2; ++i)
        {
            size_t index = i * 2;
            const Vector3f& v1 = linesLock->at(index);
            const Vector3f& v2 = linesLock->at(index + 1);

            glVertex3f(v1(0), v1(1), v1(2));
            glVertex3f(v2(0), v2(1), v2(2));
        }
    }

    glEnd();
}

void RenderLines::drawArray()
{
    glLineWidth(3);
    mRenderMaterial->glMaterial();

    // draw in array mode
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_NORMAL_ARRAY);

    {
        auto linesLock = mLines.lock();
        if (!linesLock->empty())
        {
            // assign triangle data
            glVertexPointer(3, GL_FLOAT, 0, linesLock->data());

            auto indicesLock = mIndices.lock();
            // draw triangles
            glDrawElements(GL_LINES, static_cast<GLsizei>(indicesLock->size()*2),
                           GL_UNSIGNED_INT, indicesLock->data());
        }
    }

    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_NORMAL_ARRAY);
}

void RenderLines::drawVBO()
{
    mRenderMaterial->glMaterial();
    // not implemented
}

void RenderLines::update()
{
    size_t size = mLines->size() / 2;
    {
        auto indicesLock = mIndices.lock();
        indicesLock->resize(size);
        for (size_t i = 0; i < size; ++i)
        {
            (*indicesLock)[i][0] = static_cast<unsigned int>(i * 2);
            (*indicesLock)[i][1] = static_cast<unsigned int>(i * 2 + 1);
        }
    }
}

void RenderLines::createBuffers()
{

}

void RenderLines::refreshBuffers()
{

}

void RenderLines::cleanup()
{

}

void RenderLines::initialize()
{
    update();
}

Monitor<Vectorfs>& RenderLines::getLines()
{
    return mLines;
}

//typdef Lines std::array<unsigned int, 2>>; in geometric data
Monitor<std::vector<std::array<unsigned int, 2>>>& RenderLines::getIndices()
{
    return mIndices;
}
