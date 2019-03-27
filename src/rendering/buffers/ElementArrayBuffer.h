#ifndef ELEMENTARRAYBUFFER_H
#define ELEMENTARRAYBUFFER_H

#include "Buffer.h"
#include <GL/glew.h>


class ElementArrayBuffer : public Buffer
{
public:
    ElementArrayBuffer();
    ~ElementArrayBuffer();

    void setBufferData(GLsizeiptr size, GLvoid const* ptr);

    // Buffer interface
public:
    virtual void bind();
    virtual void unbind();

private:
    GLuint mElementArrayBufferObj;
};

#endif // ELEMENTARRAYBUFFER_H
