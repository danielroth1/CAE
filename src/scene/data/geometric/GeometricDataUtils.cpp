#include "GeometricDataUtils.h"

using namespace Eigen;

Vectors GeometricDataUtils::calculateNormals(
        const Vectors& positions,
        const Faces& faces)
{
    Vectors normals;
    normals.resize(positions.size());

    for (size_t i = 0; i < positions.size(); ++i)
    {
        normals[i] = Vector::Zero();
    }

    // calculate normals for each vertex
    for (size_t i = 0; i < faces.size(); ++i)
    {
        const Face& t = faces[i];

        Vector p0 = positions[t[0]];
        Vector p1 = positions[t[1]];
        Vector p2 = positions[t[2]];

        Vector normal = (p0 - p1).cross(p0 - p2);

        normals[t[0]] += normal;
        normals[t[1]] += normal;
        normals[t[2]] += normal;

    }

    for (Vector& normal : normals)
    {
        normal.normalize();
    }
    return normals;
}

void GeometricDataUtils::updateBoundingBox(
        Vectors& positions,
        BoundingBox& boundingBox)
{
    Vector3d& boundingBoxMin = boundingBox.min();
    Vector3d& boundingBoxMax = boundingBox.max();
    for (unsigned int i = 0; i < positions.size(); ++i)
    {
        boundingBoxMin[0] = std::min(positions[i][0], boundingBoxMin[0]);
        boundingBoxMin[1] = std::min(positions[i][1], boundingBoxMin[1]);
        boundingBoxMin[2] = std::min(positions[i][2], boundingBoxMin[2]);
        boundingBoxMax[0] = std::max(positions[i][0], boundingBoxMax[0]);
        boundingBoxMax[1] = std::max(positions[i][1], boundingBoxMax[1]);
        boundingBoxMax[2] = std::max(positions[i][2], boundingBoxMax[2]);
    }
    boundingBox.mid() = 0.5 * boundingBox.min() + 0.5 * boundingBox.max();
    boundingBox.size() = boundingBox.max() - boundingBox.min();
}

GeometricDataUtils::GeometricDataUtils()
{

}
