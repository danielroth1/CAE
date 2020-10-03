#ifndef IMAGELOADER_H
#define IMAGELOADER_H

#include <memory>
#include <string>

class Image;

class ImageLoader
{
public:
    static ImageLoader* instance()
    {
        return mInstance;
    }

    ImageLoader();

    std::shared_ptr<Image> loadBMP(const std::string& filename);

private:

    // Converts a four-character array to an integer, using little-endian form.
    int toInt(const char* bytes);

    // Converts a two-character array to a short, using little-endian form.
    short toShort(const char* bytes);

    // Reads the next four bytes as an integer, using little-endian form.
    int readInt(std::ifstream& input);

    // Reads the next two bytes as a short, using little-endian form.
    short readShort(std::ifstream& input);

    static ImageLoader* mInstance;
};

#endif // IMAGELOADER_H
