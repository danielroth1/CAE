#include "StringUtils.h"

StringUtils::StringUtils()
{

}

bool StringUtils::startsWith(const std::string& target, const std::string& startingWith)
{
    if (target.size() < startingWith.size())
        return false;

    for (std::size_t i = 0; i < startingWith.size(); ++i)
    {
        if (target[i] != startingWith[i])
            return false;
    }

    return true;
}
