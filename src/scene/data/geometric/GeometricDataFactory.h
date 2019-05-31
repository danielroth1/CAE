#ifndef GEOMETRICDATAFACTORY_H
#define GEOMETRICDATAFACTORY_H

#include <data_structures/DataStructures.h>

class MeshCriteria;
class Polygon2D;
class Polygon3D;

// Create geometric data from vectors or
// convert 2D to 3D polygons
class GeometricDataFactory
{
public:

    // Creates a simple 2d box.
    static Polygon2D create2DBox(
            double width,
            double length,
            double height);

    // Creates a simple 3d box that consists of 5 tetrahedrons which is the
    // the minimum number.
    static Polygon3D create3DBox(
            double width,
            double length,
            double height);

    // Creates a 3d box that is created from a 2d box and the given mesh
    // criteria.
    static Polygon3D create3DBox(
            double width,
            double length,
            double height,
            const MeshCriteria& meshCriteria);

    // Creates a 2d mesh of a sphere.
    // Create the vertices by projecting those of a
    // icosahedron on a sphere (see wikipedia: Geodesic polyhedron).
    // The Polygon is in world space coordinates.
    // The resolution is equal to the resolution of each side of the
    // icosahedron. With a resolution of n each side consists of
    // n*2-1 triangles.
    static Polygon2D create2DSphere(
            double radius,
            int resolution);

    // 3D discretization applyed on the sphere created with
    // create2DSphere().
    static Polygon3D create3DSphere(
            double radius,
            int resolution,
            const MeshCriteria& meshCriteria);

    static Polygon3D createPolygon3DFromPolygon2D(
            Polygon2D& p,
            const MeshCriteria& meshCriteria);

protected:
    GeometricDataFactory();

private:

    // Calculates the triangles for the given vertices.
    // It is assumed that all edges are of the same length.
    // The length of an edge is equal to the minimum distance
    // of the vertices.
    static void calculateTriangles(
            const Vectors& vertices,
            Faces& trianglesOut,
            Vectors& normalsOut);

    static void insertIfNotContains(
            const std::tuple<size_t, size_t>& vertexPair,
            std::vector<size_t>& vertices);

    static void insertIfNotContains(
            size_t vertex,
            std::vector<size_t>& vertices);

    static void insertIfNotContains(
            const Eigen::Vector& vertex,
            std::vector<Eigen::Vector>& vertices);

    static unsigned int findIndexOfVertex(
            const Eigen::Vector& v,
            const std::vector<Eigen::Vector>& vertices);

    static Vectors removeDuplicatedVertices(const Vectors& vertices);

    static std::vector<std::tuple<size_t, size_t>> removeDuplicatedEdges(
            const std::vector<std::tuple<size_t, size_t>>& edges);

    static Faces removeDuplicatedTriangles(const Faces& triangles);

    static Eigen::Vector calculateCenter(const Vectors& vertices);
};

#endif // GEOMETRICDATAFACTORY_H
