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

    // calculate normals for each face
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
    boundingBox.min() = positions[0];
    boundingBox.max() = positions[0];
    for (unsigned int i = 1; i < positions.size(); ++i)
    {
        boundingBox.min() = boundingBox.min().cwiseMin(positions[i]);
        boundingBox.max() = boundingBox.max().cwiseMax(positions[i]);
    }
    boundingBox.mid() = 0.5 * boundingBox.min() + 0.5 * boundingBox.max();
    boundingBox.size() = boundingBox.max() - boundingBox.min();
}

GeometricDataUtils::GeometricDataUtils()
{

}
