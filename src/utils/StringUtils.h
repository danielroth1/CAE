#ifndef STRINGUTILS_H
#define STRINGUTILS_H


#include <string>

class StringUtils
{
public:
    StringUtils();

    static bool startsWith(const std::string& target, const std::string& startingWith);
};

#endif // STRINGUTILS_H
