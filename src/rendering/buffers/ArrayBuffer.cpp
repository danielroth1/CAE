#include "ArrayBuffer.h"

ArrayBuffer::ArrayBuffer()
{
    glGenBuffers(1, &mArrayBufferObj);
}

ArrayBuffer::~ArrayBuffer()
{
    glDeleteBuffers(1, &mArrayBufferObj);
}

void ArrayBuffer::setBufferData(GLsizeiptr size, const GLvoid* ptr)
{
    bind();

    glBufferData(GL_ARRAY_BUFFER, size, nullptr, GL_STREAM_DRAW);
    glBufferSubData(GL_ARRAY_BUFFER, 0, size, ptr);

    unbind();
}

void ArrayBuffer::bind()
{
    glBindBuffer(GL_ARRAY_BUFFER, mArrayBufferObj);
}

void ArrayBuffer::unbind()
{
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}
