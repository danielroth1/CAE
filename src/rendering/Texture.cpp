#include "Image.h"
#include "Texture.h"

#include <GL/glew.h>


Texture::Texture(const std::shared_ptr<Image>& image)
    : mImage(image)
{
    mId = generateTexture(image);
}

void Texture::bind() const
{
    glBindTexture(GL_TEXTURE_2D, mId);
}

unsigned int Texture::getId() const
{
    return mId;
}

unsigned int Texture::generateTexture(const std::shared_ptr<Image>& image) const
{
    GLuint id;
    glGenTextures(1, &id);
    glBindTexture(GL_TEXTURE_2D, id);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexImage2D(GL_TEXTURE_2D,                 // Always GL_TEXTURE_2D
                 0,                             // 0 for now
                 GL_RGB,                        // Format OpenGL uses for image
                 image->getWidth(),             // Width
                 image->getHeight(),            // Height
                 0,                             // The border of the image
                 GL_RGB,                        // GL_RGB, because pixels are stored in RGB format
                 GL_UNSIGNED_BYTE,              // GL_UNSIGNED_BYTE, because pixels are stored as unsigned numbers
                 image->getPixels().data());    // The actual pixel data
    return id;
}
