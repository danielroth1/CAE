#ifndef IOUTILS_H
#define IOUTILS_H

#include <Eigen/Core>
#include <fstream>
#include <iomanip>
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

    template <class Type>
    static std::ofstream& writeNumber(std::ofstream& stream, Type number)
    {
        stream << number;
        return stream;
    }

    template <class Type>
    static std::ofstream& writeNumber(std::ofstream& stream, Type number, int precision)
    {
        stream << std::setprecision(precision) << number;
        return stream;
    }

    template <class Type, unsigned int n>
    static std::ofstream& writeVector(std::ofstream& stream, Eigen::Matrix<Type, n, 1>& v)
    {
        stream << v(0) << " " << v(1) << " " << v(2);
        return stream;
    }

    template <class Type, unsigned int n>
    static std::ofstream& writeVector(std::ofstream& stream, Eigen::Matrix<Type, n, 1>& v, int precision)
    {
        stream << std::setprecision(precision) << v(0) << " " << v(1) << " " << v(2);
        return stream;
    }

    template <class Type, unsigned int n>
    static std::ofstream& writeArray(std::ofstream& stream, std::array<Type, n>& a)
    {
        stream << a[0] << " " << a[1] << " " << a[2];
        return stream;
    }

    template <class Type, unsigned int n>
    static std::ofstream& writeArray(std::ofstream& stream, std::array<Type, n>& a, int precision)
    {
        stream << std::setprecision(precision) << a[0] << " " << a[1] << " " << a[2];
        return stream;
    }



protected:
    IOUtils();
};

#endif // IOUTILS_H
