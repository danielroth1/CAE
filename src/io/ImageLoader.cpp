#include "ImageLoader.h"

#include <fstream>
#include <iostream>

#include <rendering/Image.h>

ImageLoader::ImageLoader()
{

}

std::shared_ptr<Image> ImageLoader::loadBMP(const char* filename)
{
    std::ifstream input;
    input.open(filename, std::ifstream::binary);
    if (input.fail())
    {
        std::cout << "Could not find file\n";
        return nullptr;
    }
    char buffer[2];
    input.read(buffer, 2);
    if (buffer[0] != 'B' || buffer[1] != 'M')
    {
        std::cout << "Not a bitmap file\n";
        return nullptr;
    }
    input.ignore(8);
    int dataOffset = readInt(input);

    // Read the header
    int headerSize = readInt(input);
    int width;
    int height;
    switch(headerSize)
    {
        case 40:
            //V3
            width = readInt(input);
            height = readInt(input);
            input.ignore(2);
            if (readShort(input) != 24)
            {
                std::cout << "Image is not 24 bits per pixel\n";
                return nullptr;
            }
            if (readShort(input) != 0)
            {
                std::cout << "Image is compressed\n";
                return nullptr;
            }
            break;
        case 12:
            // OS/2 V1
            width = readShort(input);
            height = readShort(input);
            input.ignore(2);
            if (readShort(input) != 24)
            {
                std::cout << "Image is not 24 bits per pixel\n";
                return nullptr;
            }
            break;
        case 64:
            // OS/2 V2
            std::cout << "Can't load OS/2 V2 bitmaps\n";
            return nullptr;
        case 108:
            // Windows V4
            std::cout << "Can't load Windows V4 bitmaps\n";
            return nullptr;
        case 124:
            // Windows V5
            std::cout << "Can't load Windows V5 bitmaps\n";
            return nullptr;
        default:
            std::cout << "Unknown bitmap format\n";
            return nullptr;
    }

    // Read the data
    int bytesPerRow = ((width * 3 + 3) / 4) * 4 - (width * 3 % 4);
    int size = bytesPerRow * height;
    std::vector<char> pixels;
    pixels.resize(static_cast<std::size_t>(size));
    input.seekg(dataOffset, std::ios_base::beg);
    input.read(pixels.data(), size);

    // Get the data into the right format
    std::vector<char> pixels2;
    pixels2.resize(static_cast<std::size_t>(width * height * 3));
    for(int y = 0; y < height; y++) {
        for(int x = 0; x < width; x++) {
            for(int c = 0; c < 3; c++) {
                pixels2[static_cast<std::size_t>(3 * (width * y + x) + c)] =
                    pixels[static_cast<std::size_t>(bytesPerRow * y + 3 * x + (2 - c))];
            }
        }
    }

    input.close();
    return std::make_shared<Image>(pixels2, width, height);
}


int ImageLoader::toInt(const char* bytes) {
    return static_cast<int>(
                ((static_cast<unsigned char>(bytes[3]) << 24) |
                 (static_cast<unsigned char>(bytes[2]) << 16) |
                 (static_cast<unsigned char>(bytes[1]) << 8) |
                 static_cast<unsigned char>(bytes[0]))
            );
}

short ImageLoader::toShort(const char* bytes) {
    return static_cast<short>(
                ((static_cast<unsigned char>(bytes[1]) << 8) |
                   static_cast<unsigned char>(bytes[0]))
            );
}

int ImageLoader::readInt(std::ifstream &input) {
    char buffer[4];
    input.read(buffer, 4);
    return toInt(buffer);
}

short ImageLoader::readShort(std::ifstream &input) {
    char buffer[2];
    input.read(buffer, 2);
    return toShort(buffer);
}

ImageLoader* ImageLoader::mInstance = new ImageLoader();
