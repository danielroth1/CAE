#ifndef TEXTUREUTILS_H
#define TEXTUREUTILS_H

#include <Eigen/Core>
#include <iostream>

class TextureUtils
{
public:

    // types are numerical types like float or double
    // RT - return type
    // IT - input type
    template <class RT, class IT>
    static std::vector<Eigen::Matrix<RT, 2, 1>> createSpericalTextureCoordinates(
            std::vector<Eigen::Matrix<IT, 3, 1>> coordinates)
    {
        Eigen::Matrix<IT, 3, 1> center = Eigen::Matrix<IT, 3, 1>::Zero();
        for (const Eigen::Matrix<IT, 3, 1>& v : coordinates)
        {
            center += v;
        }
        center /= coordinates.size();

        std::vector<Eigen::Matrix<RT, 2, 1>> texCoords;
        for (size_t i = 0; i < coordinates.size(); ++i)
        {
            Eigen::Matrix<IT, 3, 1> ver = coordinates[i] - center;
            RT u = static_cast<RT>(1.0f / (2 * M_PI) * atan2f(ver(2), ver(0)) + 0.5f);
            RT v = static_cast<RT>(-1.0f / M_PI * asinf(ver(1)) + 0.5f);
//            RT u =
            texCoords.push_back(Eigen::Matrix<RT, 2, 1>(u, v));
        }

        return texCoords;
    }

protected:
    TextureUtils();


};

#endif // TEXTUREUTILS_H
