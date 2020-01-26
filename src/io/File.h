#ifndef FILE_H
#define FILE_H

#include <fstream>
#include <string>
#include <vector>

// File class that provides basic functionality to access files
// and move in directories. Could be faster but file system access
// is usually not time critical.
class File
{
public:
    File(std::string path);

    bool exists() const;

    std::string getPath() const;
    std::string getRelativePath() const;
    std::string getName() const;

    // If a file is selected, returns the extension of the file where
    // <path>/<file-name><file-extension>
    // -> includes the ".", e.g. if path = "/urs/admin/objs/test.obj" then
    // this method returns ".obj".
    std::string getExtension() const;

    std::vector<std::string> getPathParts() const;

    // Returns true if the directory exists.
    bool goUpDirectory();

    // Returns true if the directory exists and the operation
    // was sucessfull.
    bool enterDirectiory(std::string dir);

    std::ifstream read(bool& successful) const;

    static const char SEPARATOR;


private:
    void update(std::string path);

    std::string mPath;
    std::vector<std::string> mPathParts;
    std::string mRelativePath;
    std::string mNameWithExtension;
    std::string mExtension;
    std::string mName;

};

#endif // FILE_H
