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

    bool exists();

    std::string getPath() const;
    std::string getRelativePath() const;
    std::string getName() const;
    std::vector<std::string> getPathParts() const;

    // Returns true if the directory exists.
    bool goUpDirectory();

    // Returns true if the directory exists and the operation
    // was sucessfull.
    bool enterDirectiory(std::string dir);

    std::ifstream read(bool& successful);

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
