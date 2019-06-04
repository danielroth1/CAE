#ifndef IMAGE_H
#define IMAGE_H

#include <vector>

class Image
{
public:
    Image(std::vector<char> pixels,
          int width,
          int height);

    const std::vector<char>& getPixels() const;
    void setPixels(const std::vector<char>& pixels);

    int getWidth() const;
    void setWidth(int width);

    int getHeight() const;
    void setHeight(int height);

private:
    std::vector<char> mPixels;
    int mWidth;
    int mHeight;
};

#endif // IMAGE_H
