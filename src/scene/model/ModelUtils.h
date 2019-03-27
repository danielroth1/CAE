#ifndef MODELUTILS_H
#define MODELUTILS_H

// Includes
#include <data_structures/DataStructures.h>

// ModelUtils contains functionality that is shared between different
// Models.
class ModelUtils
{
public:
    ModelUtils();

    // Calculates the normals according to some normal
    // calculation strategy (TODO).
    // Possible
    // \param positions - input positions
    // \param faces - input faces
    // \param normals - output normals, is initialized and
    //      of the same size as positions (call resize
    //      on a newly created vector).
    template<class Type>
    static std::vector<Eigen::Matrix<Type, 3, 1>>
    calculateNormals(
            std::vector<Eigen::Matrix<Type, 3, 1>>& positions,
            Faces& faces,
            std::vector<Eigen::Matrix<Type, 3, 1>>& normals)
    {
        normals.resize(positions.size());
        for (size_t i = 0; i < positions.size(); ++i)
        {
            normals[i] = Eigen::Matrix<Type, 3, 1>::Zero();
        }

        // calculate normals for each vertex
        for (size_t i = 0; i < faces.size(); ++i)
        {
            Face& t = faces[i];

            // face normals
            Eigen::Matrix<Type, 3, 1> p0 = positions[t[0]];
            Eigen::Matrix<Type, 3, 1> p1 = positions[t[1]];
            Eigen::Matrix<Type, 3, 1> p2 = positions[t[2]];

            Eigen::Matrix<Type, 3, 1> normal = (p0 - p1).cross(p0 - p2);

//            normals[t[0]] += normal;
//            normals[t[1]] += normal;
//            normals[t[2]] += normal;

            normal.normalize();

            // angles
            float angle_factor = std::acos((p0 - p1).normalized().dot((p0 - p2).normalized()));
            Eigen::Vector3f angle_normal = angle_factor * normal;
            normals[t[0]] += angle_normal;

            angle_factor = std::acos((p1 - p0).normalized().dot((p1 - p2).normalized()));
            angle_normal = angle_factor * normal;
            normals[t[1]] += angle_normal;

            angle_factor = std::acos((p2 - p0).normalized().dot((p2 - p1).normalized()));
            angle_normal = angle_factor * normal;
            normals[t[2]] += angle_normal;

        }

        for (Eigen::Matrix<Type, 3, 1>& normal : normals)
        {
            normal.normalize();
        }
        return normals;
    }
};

#endif // MODELUTILS_H
