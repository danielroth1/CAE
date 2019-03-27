#ifndef RENDERERUTILS_H
#define RENDERERUTILS_H

#include <GL/glew.h>

class RendererUtils
{
public:

    static GLuint createVBO();

    static void refreshVBO(
            GLuint id,
            const void* data,
            int dataSize,
            GLenum target,
            GLenum usage);
};

#endif // RENDERERUTILS_H
