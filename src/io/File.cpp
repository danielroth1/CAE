#include "File.h"

#include <boost/filesystem.hpp>
#include <iostream>

const char File::SEPARATOR = boost::filesystem::path::preferred_separator;

File::File(std::string path)
    : mPath(path)
{
    update(path);
}

bool File::exists()
{
    return boost::filesystem::exists(boost::filesystem::path(mPath));
}

std::string File::getPath() const
{
    return mPath;
}

std::string File::getRelativePath() const
{
    return mRelativePath;
}

std::string File::getName() const
{
    return mName;
}

std::vector<std::string> File::getPathParts() const
{
    return mPathParts;
}

bool File::goUpDirectory()
{
    std::string parentDir = "";
    for (size_t i = 0; i < mPathParts.size()-1; ++i)
    {
        parentDir += mPathParts[i];
        if (i < mPathParts.size() - 2)
            parentDir += SEPARATOR;
    }
    update(parentDir);
    return exists();
}

bool File::enterDirectiory(std::string dir)
{
    if (exists())
        return false;
    update(mPath + SEPARATOR + dir);
    return exists();
}

std::ifstream File::read(bool& successful)
{
    std::ifstream in(getPath());

    if (!exists())
    {
        std::cout << "Can not open file because it does not exist: " << getPath() << "\n";
        successful = false;
        return in;
    }
    if (!in.is_open())
    {
        std::cout << "Can not open file although it exists. Missing privilegs? " << getPath() << "\n";
        successful = false;
    }
    successful = true;
    return in;
}

void File::update(std::string path)
{
    mPath = path;

    // split up path
    mPathParts.clear();
    for (auto& part : boost::filesystem::path(mPath))
    {
        if (part == "/")
            mPathParts.push_back("");
        else
            mPathParts.push_back(part.c_str());
    }

    // relative path
    mRelativePath = "";
    for (size_t i = 0; i < mPathParts.size(); ++i)
    {
        mRelativePath += mPathParts[i];
        if (i < mPathParts.size() - 1)
            mRelativePath += SEPARATOR;
    }

    // name with extension
    mNameWithExtension = mPathParts[mPathParts.size()-1];

    // name
    mExtension = boost::filesystem::extension(mNameWithExtension);

    // extension
    mName = boost::filesystem::basename(mNameWithExtension);
}
