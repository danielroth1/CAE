#include "UniformBuffer.h"

UniformBuffer::UniformBuffer()
{
    glGenBuffers(1, &mUniformBufferObj);

}

UniformBuffer::UniformBuffer(GLsizeiptr size)
{
    glBindBuffer(GL_UNIFORM_BUFFER, mUniformBufferObj);
    glBufferData(GL_UNIFORM_BUFFER, size,
        reinterpret_cast<GLfloat*>(0), GL_DYNAMIC_DRAW);
    glBindBuffer(GL_UNIFORM_BUFFER, 0);
}

UniformBuffer::~UniformBuffer()
{
    glDeleteBuffers(1, &mUniformBufferObj);
}

void UniformBuffer::bindBufferBase(GLuint index)
{
    glBindBufferBase(GL_UNIFORM_BUFFER, index, mUniformBufferObj);
}

void UniformBuffer::bind()
{
    glBindBuffer(GL_UNIFORM_BUFFER, mUniformBufferObj);
}

void UniformBuffer::unbind()
{
    glBindBuffer(GL_UNIFORM_BUFFER, 0);
}
