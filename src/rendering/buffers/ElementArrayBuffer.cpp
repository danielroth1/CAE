#include "ElementArrayBuffer.h"

ElementArrayBuffer::ElementArrayBuffer()
{
    glGenBuffers(1, &mElementArrayBufferObj);
}

ElementArrayBuffer::~ElementArrayBuffer()
{
    glDeleteBuffers(1, &mElementArrayBufferObj);
}

void ElementArrayBuffer::setBufferData(GLsizeiptr size, const GLvoid* ptr)
{
    bind();

    glBufferData(GL_ELEMENT_ARRAY_BUFFER, size, nullptr, GL_STREAM_DRAW);
    glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, 0, size, ptr);

    unbind();
}

void ElementArrayBuffer::bind()
{
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mElementArrayBufferObj);
}

void ElementArrayBuffer::unbind()
{
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}
