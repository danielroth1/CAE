#include "File.h"

#include <boost/filesystem.hpp>

const char File::SEPARATOR = boost::filesystem::path::preferred_separator;

File::File(std::string path)
    : mPath(path)
{

}

bool File::exists()
{
    return boost::filesystem::exists(boost::filesystem::path(mPath));
}

std::string File::getPath()
{
    return mPath;
}

std::string File::getRelativePath()
{
    return mRelativePath;
}

std::string File::getName()
{
    return mName;
}

std::vector<std::string> File::getPathParts()
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

void File::update(std::string path)
{
    mPath = path;

    // split up path
    mPathParts.clear();
    for (auto& part : boost::filesystem::path(mPath))
    {
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
