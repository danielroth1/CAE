#include "RendererUtils.h"

#include <iostream>


GLuint RendererUtils::createVBO()
{
    // 0 is reserved, glGenBuffers() will return non-zero id if success
    GLuint id = 0;
    // create a vbo
    glGenBuffers(1, &id);
    return id;
}

void RendererUtils::refreshVBO(
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
