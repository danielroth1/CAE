#ifndef TEXTURE_H
#define TEXTURE_H

#include <memory>

class Image;

class Texture
{
public:
    Texture(const std::shared_ptr<Image>& image);

    // Binds the texture by calling the appropriate OpenGL function.
    // Calls glBindTexture().
    void bind();

    // Returns the OpenGL texture id.
    unsigned int getId() const;

private:


    // Initializes the texture. Call this method only within the OpenGL context.
    // \return the image id
    unsigned int generateTexture(const std::shared_ptr<Image>& image);

    std::shared_ptr<Image> mImage;
    unsigned int mId;
    bool mGenerated;
};

#endif // TEXTURE_H
