#ifndef RENDERPOLYGON2D_H
#define RENDERPOLYGON2D_H

// Includes
#include <GL/glew.h>

#include "RenderObject.h"
#include <data_structures/DataStructures.h>
#include <multi_threading/Monitor.h>

#include <rendering/buffers/BufferedData.h>


// Forward Declarations
class RenderObjectVisitor;

// RenderPolygon2D
class RenderPolygon2D : public RenderObject
{
public:

    // make this class independent of SceneObject

    RenderPolygon2D();

    virtual ~RenderPolygon2D() override;

    // Visitor
    void accept(RenderObjectVisitor& visitor) override;

    virtual void draw() override;

    virtual void drawImmediate() override;

    virtual void drawArray() override;

    virtual void drawVBO() override;

    // Updates the vertex positions in the buffer.
    void update() override;

    void createBuffers() override;

    /*
     * Update position and vertex buffers by copying the
     * data from their respecitve vectors in the buffer.
     */
    void refreshBuffers() override;

    GLuint createVBO();

    void refreshVBO(
            GLuint id,
            const void* data,
            int dataSize,
            GLenum target,
            GLenum usage);

    void cleanup() override;

    void initialize() override;

    // Resets the transformation matrix
    void resetTransform();

    size_t getNumberOfPositions();
    size_t getNumberOfTriangles();

    Monitor<Vectorfs>& getPositions();
    Monitor<Vectorfs>& getNormals();
    Monitor<Faces>& getFaces();

protected:

    bool mVBOsupported;
    GLuint mVao;

private:

    BufferedData<Eigen::Vectorf, float, 3> mPositions;
    BufferedData<Eigen::Vectorf, float, 3> mNormals;
    BufferedData<Face, unsigned int, 3> mFaces;
};

#endif // RENDERPOLYGON2D_H
