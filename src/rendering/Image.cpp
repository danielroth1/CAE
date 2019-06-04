#include "Image.h"

Image::Image(
        std::vector<char> pixels,
        int width,
        int height)
    : mPixels(pixels)
    , mWidth(width)
    , mHeight(height)
{

}

const std::vector<char>& Image::getPixels() const
{
    return mPixels;
}

void Image::setPixels(const std::vector<char>& pixels)
{
    mPixels = pixels;
}

int Image::getWidth() const
{
    return mWidth;
}

void Image::setWidth(int width)
{
    mWidth = width;
}

int Image::getHeight() const
{
    return mHeight;
}

void Image::setHeight(int height)
{
    mHeight = height;
}
