#ifndef MODELUTILS_H
#define MODELUTILS_H

// Includes
#include <data_structures/DataStructures.h>

#include <scene/data/geometric/PolygonTopology.h>

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
            const std::vector<Eigen::Matrix<Type, 3, 1>>& positions,
            const Faces& faces,
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
            const Face& t = faces[i];

            // face normals
            const Eigen::Matrix<Type, 3, 1>& p0 = positions[t[0]];
            const Eigen::Matrix<Type, 3, 1>& p1 = positions[t[1]];
            const Eigen::Matrix<Type, 3, 1>& p2 = positions[t[2]];

            Eigen::Matrix<Type, 3, 1> normal = (p0 - p1).cross(p0 - p2);

//            normals[t[0]] += normal;
//            normals[t[1]] += normal;
//            normals[t[2]] += normal;

            normal.normalize();

            // angles
            Type angle_factor = std::acos((p0 - p1).normalized().dot((p0 - p2).normalized()));
            Eigen::Matrix<Type, 3, 1> angle_normal = angle_factor * normal;
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

    // Calculates the vertex normals by averaging for each vertex the neighboring
    // face normals.
    //\param faceNormals - face normals that can be calculated with
    //          calcualteFaceNormals()
    //\param normals - returned vertex normals
    template<class Type>
    static std::vector<Eigen::Matrix<Type, 3, 1>>
    calculateVertexFromFaceNormals(
            const std::vector<Eigen::Matrix<Type, 3, 1>>& positions,
            const Faces& faces,
            const std::vector<Eigen::Matrix<Type, 3, 1>>& faceNormals,
            const PolygonTopology& topology,
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
            const Face& t = faces[i];

            normals[t[0]] += faceNormals[i];
            normals[t[1]] += faceNormals[i];
            normals[t[2]] += faceNormals[i];
        }

        for (size_t i = 0; i < positions.size(); ++i)
        {
            normals[i] /= topology.getVertices()[i].getFaceIds().size();
        }
//        for (Eigen::Matrix<Type, 3, 1>& normal : normals)
//        {
//            normal.normalize();
//        }
        return normals;
    }

    template<class Type>
    static std::vector<Eigen::Matrix<Type, 3, 1>>
    calculateFaceNormals(
            const std::vector<Eigen::Matrix<Type, 3, 1>>& positions,
            const Faces& faces,
            std::vector<Eigen::Matrix<Type, 3, 1>>& normals)
    {
        // calculate normals for each face
        normals.resize(faces.size());
        for (size_t i = 0; i < faces.size(); ++i)
        {
            const Face& t = faces[i];

            // face normals
            const Eigen::Matrix<Type, 3, 1>& p0 = positions[t[0]];
            const Eigen::Matrix<Type, 3, 1>& p1 = positions[t[1]];
            const Eigen::Matrix<Type, 3, 1>& p2 = positions[t[2]];

            normals[i] = (p0 - p1).cross(p0 - p2).normalized();
        }
        return normals;
    }
};

#endif // MODELUTILS_H
