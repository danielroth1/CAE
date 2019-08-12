#ifndef IOUTILS_H
#define IOUTILS_H

#include <Eigen/Core>
#include <sstream>

class IOUtils
{
public:

    template <class Type>
    static Type readNumber(std::stringstream& ss)
    {
        Type num;
        ss >> num;
        return num;
    }

    template <class Type>
    static void readNumber(std::stringstream& ss, Type& numOut)
    {
        ss >> numOut;
        return numOut;
    }

    template <class Type, unsigned int n>
    static void readVector(std::stringstream& ss, Eigen::Matrix<Type, n, 1>& vOut)
    {
        for (size_t i = 0; i < n; ++i)
            ss >> vOut(i);
    }

    template <class Type, unsigned int n>
    static Eigen::Matrix<Type, n, 1> readVector(std::stringstream& ss)
    {
        Eigen::Matrix<Type, n, 1> v;
        for (size_t i = 0; i < n; ++i)
            ss >> v(i);
        return v;
    }

    template <class Type, unsigned int n>
    static void readArray(std::stringstream& ss, std::array<Type, n>& aOut)
    {
        for (size_t i = 0; i < n; ++i)
            ss >> aOut[i];
    }

    template <class Type, unsigned int n>
    static std::array<Type, n> readArray(std::stringstream& ss)
    {
        std::array<Type, n> a;
        for (size_t i = 0; i < n; ++i)
            ss >> a[i];
        return a;
    }

protected:
    IOUtils();
};

#endif // IOUTILS_H
