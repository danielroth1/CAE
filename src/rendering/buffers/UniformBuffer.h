#ifndef UNIFORMBUFFER_H
#define UNIFORMBUFFER_H

#include <GL/glew.h>
#include "Buffer.h"



class UniformBuffer : public Buffer
{
public:
    UniformBuffer();
    UniformBuffer(GLsizeiptr size);

    ~UniformBuffer();

    void bindBufferBase(GLuint index);

    // Buffer interface
public:
    virtual void bind();
    virtual void unbind();

private:
    GLuint mUniformBufferObj;
};

#endif // UNIFORMBUFFER_H
