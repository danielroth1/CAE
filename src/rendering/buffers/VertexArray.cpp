#include "VertexArray.h"

VertexArray::VertexArray()
{
    glGenVertexArrays(1, &mVertexArrayObj);
}

VertexArray::~VertexArray()
{
    glDeleteBuffers(1, &mVertexArrayObj);
}

void VertexArray::bind()
{
    glBindVertexArray(mVertexArrayObj);
}

void VertexArray::unbind()
{
    glBindVertexArray(0);
}
