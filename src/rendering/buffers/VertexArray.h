#ifndef VERTEXARRAY_H
#define VERTEXARRAY_H

#include "Buffer.h"
#include <GL/glew.h>


class VertexArray : public Buffer
{
public:
    VertexArray();
    ~VertexArray();

    // Buffer interface
public:
    virtual void bind();
    virtual void unbind();

private:
    GLuint mVertexArrayObj;
};

#endif // VERTEXARRAY_H
