#include "RenderObjectVisitor.h"
#include "RenderPolygon2D.h"
#include <GL/glut.h>

#include <iostream>
#include <QDebug>
#include <times/timing.h>

RenderPolygon2D::RenderPolygon2D()
    : RenderObject()
    , mPositions(GL_ARRAY_BUFFER, GL_DYNAMIC_DRAW)
    , mNormals(GL_ARRAY_BUFFER, GL_DYNAMIC_DRAW)
    , mFaces(GL_ELEMENT_ARRAY_BUFFER, GL_STATIC_DRAW)
{
    mVao = 0;
    mTransform.setIdentity();
}

RenderPolygon2D::~RenderPolygon2D()
{

}

void RenderPolygon2D::accept(RenderObjectVisitor& visitor)
{
    visitor.visit(*this);
}

/*
 * Draws this scene object.
 * Requires the normals to be updated if not already done.
 * Do this by calling "updateNormals".
 * Additionally, if the content of "m_positions" changed,
 * update the buffers with "refreshBuffers".
 */
void RenderPolygon2D::draw()
{
    drawVBO();
}

void RenderPolygon2D::drawImmediate()
{
    mRenderMaterial.glMaterial();
    glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
    glPushMatrix();
    glLoadMatrixf(mTransform.data());

    glBegin(GL_TRIANGLES);

    auto positionsLock = mPositions.getData().lock();
    auto normalsLock = mNormals.getData().lock();
    auto facesLock = mFaces.getData().lock();

        for (const Face& f : *facesLock)
        {
            for (unsigned int i = 0; i < 3; ++i)
            {
                unsigned int index = f[i];
                const Eigen::Vector3f& v = (*positionsLock)[index];
                const Eigen::Vector3f& n = (*normalsLock)[index];

                glNormal3f(n(0), n(1), n(2));
                glVertex3f(v(0), v(1), v(2));
            }
        }

    glEnd();
    glPopMatrix();
}

void RenderPolygon2D::drawArray()
{
    if (mFaces.getData().lock()->size() == 0)
        return;

    mRenderMaterial.glMaterial();
    glPushMatrix();
        glMultMatrixf(mTransform.data());

        // draw in array mode
        glEnableClientState(GL_VERTEX_ARRAY);
        glEnableClientState(GL_NORMAL_ARRAY);

        // assign triangle data
        {
            auto positionsLock = mPositions.getData().lock();
            glVertexPointer(3, GL_FLOAT, 0, positionsLock->data());

        }

        // assign normals
        {
            auto normalsLock = mNormals.getData().lock();
            glNormalPointer(GL_FLOAT, 0, normalsLock->data());
        }

//    std::cout << mTransform.matrix() << "\n\n";
    // draw triangles
        {
            auto facesLock = mFaces.getData().lock();
            glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(facesLock->size()*3),
                           GL_UNSIGNED_INT, facesLock->data());
        }

        glDisableClientState(GL_VERTEX_ARRAY);
        glDisableClientState(GL_NORMAL_ARRAY);

    glPopMatrix();
}

void RenderPolygon2D::createBuffers()
{
//    glewInit();

    // check if VBOs are supported
    if (glGenBuffers && glBindBuffer && glBufferData && glBufferSubData
            && glMapBuffer && glUnmapBuffer && glDeleteBuffers
            && glGetBufferParameteriv)
        mVBOsupported = true;
    else
    {
        mVBOsupported = false;
        qDebug() << "VBOs are not supported!";
    }

    // check if VBOs are supported
    if (glGenBuffers && glBindBuffer && glBufferData && glBufferSubData
            && glMapBuffer && glUnmapBuffer && glDeleteBuffers
            && glGetBufferParameteriv)
        mVBOsupported = true;
    else
    {
        mVBOsupported = false;
        qDebug() << "VBOs are not supported!";
    }

    START_TIMING_RENDERING("create buffers")
    // this hsould allocate the necessary memory
    mPositions.createBuffer();
    mNormals.createBuffer();
    mFaces.createBuffer();

    STOP_TIMING_RENDERING;
    START_TIMING_RENDERING("refreshBuffers()");
    refreshBuffers();
    STOP_TIMING_RENDERING;

}

void RenderPolygon2D::refreshBuffers()
{
    // Note on the usage of glBufferSubData:
    // do not write the data directly with glBufferData to
    // avoid errors occuring when graphics card (when drawing)
    // and cpu concurrently access the buffer

    mPositions.refreshBuffer();
    mNormals.refreshBuffer();
    mFaces.refreshBuffer();
}

GLuint RenderPolygon2D::createVBO()
{
    // 0 is reserved, glGenBuffers() will return non-zero id if success
    GLuint id = 0;
    // create a vbo
    glGenBuffers(1, &id);
    return id;
}

void RenderPolygon2D::refreshVBO(
        GLuint id,
        const void* data,
        int dataSize,
        GLenum target,
        GLenum usage)
{
    // activate vbo id to use
    glBindBuffer(target, id);
    // upload data to video card
    glBufferData(target, dataSize, data, usage);
    // check data size in VBO is same as input array, if not return 0 and delete VBO
    int bufferSize = 0;
    glGetBufferParameteriv(target, GL_BUFFER_SIZE, &bufferSize);
    if(dataSize != bufferSize)
    {
        glDeleteBuffers(1, &id);
        id = 0;
        std::cout << "createVBO() ERROR: Data size (" <<
                     dataSize << ") is mismatch with input array (" <<
                     bufferSize << ")." << std::endl;
    }
    // unbind after copying data
    glBindBuffer(target, 0);
}

void RenderPolygon2D::drawVBO()
{
    START_TIMING_RENDERING("RenderPolygon2D::drawVBO2")
    if (mVBOsupported == false)
        return;
    if (!mPositions.isInitialized() ||
        !mNormals.isInitialized() ||
        !mFaces.isInitialized())
        return;

    mRenderMaterial.glMaterial();
    glPushMatrix();
        glMultMatrixf(mTransform.data());
        glEnableClientState(GL_NORMAL_ARRAY);
        glEnableClientState(GL_VERTEX_ARRAY);
        mNormals.bindBuffer();
        glNormalPointer(GL_FLOAT, 0, nullptr);
        mPositions.bindBuffer();
        glVertexPointer(3, GL_FLOAT, 0, nullptr);
        mFaces.bindBuffer();
        glDrawElements(GL_TRIANGLES,
                       static_cast<int>(
                           3 * mFaces.getData().unsafe().size()),
                       GL_UNSIGNED_INT,
                       nullptr);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
        glDisableClientState(GL_VERTEX_ARRAY);
        glDisableClientState(GL_NORMAL_ARRAY);
    glPopMatrix();
    STOP_TIMING_RENDERING
}

void RenderPolygon2D::update()
{
    // TODO: implement this
}

void RenderPolygon2D::cleanup()
{
    if (mVBOsupported)
    {
        mPositions.cleanup();
        mNormals.cleanup();
        mFaces.cleanup();
    }
    mVao = 0;
}

void RenderPolygon2D::initialize()
{
    cleanup();
    createBuffers();
}

void RenderPolygon2D::resetTransform()
{
    mTransform.setIdentity();
}

size_t RenderPolygon2D::getNumberOfPositions()
{
    return mPositions.getData().unsafe().size();
}

size_t RenderPolygon2D::getNumberOfTriangles()
{
    return mFaces.getData().unsafe().size();
}

Monitor<Vectorfs>& RenderPolygon2D::getPositions()
{
    return mPositions.getData();
}

Monitor<Vectorfs>& RenderPolygon2D::getNormals()
{
    return mNormals.getData();
}

Monitor<Faces>& RenderPolygon2D::getFaces()
{
    return mFaces.getData();
}
