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
    void bind() const;

    // Returns the OpenGL texture id.
    unsigned int getId() const;

private:


    // Initializes the texture. Call this method only within the OpenGL context.
    // \return the image id
    unsigned int generateTexture(const std::shared_ptr<Image>& image) const;

    std::shared_ptr<Image> mImage;
    unsigned int mId;
};

#endif // TEXTURE_H
