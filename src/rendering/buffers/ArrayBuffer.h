#ifndef ARRAYBUFFER_H
#define ARRAYBUFFER_H

#include "Buffer.h"
#include <GL/glew.h>


class ArrayBuffer : public Buffer
{
public:
    ArrayBuffer();
    ~ArrayBuffer();

    void setBufferData(GLsizeiptr size, GLvoid const* ptr);

    // Buffer interface
public:
    virtual void bind();
    virtual void unbind();

private:
    GLuint mArrayBufferObj;
};

#endif // ARRAYBUFFER_H
