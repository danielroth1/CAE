

// Inlcudes
#include "GeometricDataFactory.h"
#include "Polygon2DTopology.h"
#include "Polygon3DTopology.h"
#include "TopologyFactory.h"

#include "modules/mesh_converter/MeshConverter.h"
#include "scene/data/geometric/Polygon2D.h"
#include "scene/data/geometric/Polygon3D.h"

#include <iostream>
#include <set>
#include <vector>

#include <modules/mesh_converter/MeshCriteria.h>

using namespace Eigen;

Polygon2D GeometricDataFactory::create2DBox(double width, double length, double height)
{
    Vectors positions;
    Faces faces;

    positions.push_back(Vector(-1,-1,-1));
    positions.push_back(Vector( 1,-1,-1));
    positions.push_back(Vector( 1, 1,-1));
    positions.push_back(Vector(-1, 1,-1));
    positions.push_back(Vector(-1,-1, 1));
    positions.push_back(Vector( 1,-1, 1));
    positions.push_back(Vector( 1, 1, 1));
    positions.push_back(Vector(-1, 1, 1));

    for (Vector& p : positions)
    {
        p[0] *= 0.5 * width;
        p[1] *= 0.5 * height;
        p[2] *= 0.5 * length;
    }

    // gather triangles and normals
    Vectors normals;
    calculateTriangles(positions, faces, normals);

    // front
    faces.push_back(Face({ {0,2,1} }));
    faces.push_back(Face({ {0,3,2} }));

    // bottom
    faces.push_back(Face({ {0,5,4} }));
    faces.push_back(Face({ {0,1,5} }));

    // left
    faces.push_back(Face({ {0,4,7} }));
    faces.push_back(Face({ {0,7,3} }));

    // top
    faces.push_back(Face({ {3,7,2} }));
    faces.push_back(Face({ {2,7,6} }));

    // right
    faces.push_back(Face({ {1,2,5} }));
    faces.push_back(Face({ {5,2,6} }));

    // back
    faces.push_back(Face({ {5,6,7} }));
    faces.push_back(Face({ {4,5,7} }));

    return Polygon2D(positions, faces);
}

Polygon3D GeometricDataFactory::create3DBox(double width, double length, double height)
{
    Polygon2D p2 = create2DBox(width, length, height);
    Faces faces = p2.getTopology2D().getFacesIndices();
    faces.push_back(Face({ {0,2,5} }));
    faces.push_back(Face({ {0,2,7} }));
    faces.push_back(Face({ {0,5,7} }));
    faces.push_back(Face({ {2,5,7} }));
    Cells cells;
    cells.push_back(Cell({ {0,1,2,5} }));
    cells.push_back(Cell({ {0,2,3,7} }));
    cells.push_back(Cell({ {0,4,5,7} }));
    cells.push_back(Cell({ {2,5,6,7} }));
    cells.push_back(Cell({ {0,7,5,2} }));

    std::shared_ptr<Polygon3DTopology> t3 =
            TopologyFactory::createPolygon3DTopology(
                faces,
                p2.getTopology2D().getFacesIndices(),
                cells,
                p2.getPositions().size());

    return Polygon3D(p2.getPositions(), t3);
}

Polygon3D GeometricDataFactory::create3DBox(double width,
                                             double length,
                                             double height,
                                             const MeshCriteria& meshCriteria)
{
    Polygon2D p2temp = create2DBox(width, length, height);
    Polygon3D p3 = createPolygon3DFromPolygon2D(p2temp, meshCriteria);
    return p3;
}

Polygon2D GeometricDataFactory::create2DSphere(
        double radius,
        int resolution)
{
    std::cout << "creating 2D sphere, radius = " << radius << ", resolution = " << resolution << "\n";
    // create vertices:
    //
    // The parametric equation of a sphere is not used
    // for discretization because the resulting triangles
    // are not equally distributed on the sphere boundary.
    // Triangles in the middle are wider and triangles closer
    // to the top or bottom are thinner.
//    // parametric equation of a sphere
//    double longitude = 0.0;
//    double colatitude = 0.0;
//    // x = r cos(longitude) sin(colatitude)
//    // y = r sin(longitude) sin(colatitude)
//    // z = r cos(colatitude)


    // First create the vertices of a icosahedron which are the
    // cyclic permutations of
    // (+-1, 0, +-phi) where phi = (1 + sqrt(5)) / 2
    // (see wikipedia: Regular icosahedron#Cartesian coordinates)

    std::vector<Vector> vertices;
    double phi = (1.0 + std::sqrt(5.0)) / 2.0;

    // permutations of (1, 0, phi)
    vertices.push_back(Vector(1, 0, phi));
    vertices.push_back(Vector(phi, 1, 0));
    vertices.push_back(Vector(0, phi, 1));

    // permutations of (-1, 0, phi)
    vertices.push_back(Vector(-1, 0, phi));
    vertices.push_back(Vector(phi, -1, 0));
    vertices.push_back(Vector(0, phi, -1));

    // permutations of (1, 0, -phi)
    vertices.push_back(Vector(1, 0, -phi));
    vertices.push_back(Vector(-phi, 1, 0));
    vertices.push_back(Vector(0, -phi, 1));

    // permutations of (-1, 0, -phi)
    vertices.push_back(Vector(-1, 0, -phi));
    vertices.push_back(Vector(-phi, -1, 0));
    vertices.push_back(Vector(0, -phi, -1));

    // triangles
    // Triangles are gathered by utilizing the fact that all edges
    // have the same length.

    // gather triangles and normals
    Faces triangles;
    Vectors normals;
    calculateTriangles(vertices, triangles, normals);

    //return Polygon2D(vertices, triangles);

    // n-frequency subdivision
    // The accuracy can be increased by subdividing the triangles
    // and thereby increasing the total number of triangles. Each
    // triangle is subdivided in the same way to avoid differently
    // sized triangles.

    int n = resolution;

    Vectors subdivision;
    for (const Face& f : triangles)
    {
        // create a subdivision for each triangle
        Vector v0 = vertices[f[0]];
        Vector v1 = vertices[f[1]];
        Vector v2 = vertices[f[2]];

        // The idea is lay a grid on the triangle and iterate over
        // the grid positions. 2D coordinates are assigned on the 2D
        // triangle plane. A grid point (x, y) on the triangle plane
        // corresponds to the 3D world space coordinate in the
        // following way:
        // (x, y) -> v_0 + x * v_x + y * v_y
        // with
        // v_x = (v_1 - v_0).norm() / n * (v_1 - v_0)
        // v_y = (v_2 - v_0).norm() / n * (v_2 - v_0)
        //
        // The grid point is on the triangle for as long as
        // x = 0, ..., n
        // y = 0, ..., n
        // and x + y <= n
        Vector v_x = 1.0 / n * (v1 - v0);
        Vector v_y = 1.0 / n * (v2 - v0);

        for (int y = 0; y <= n; ++y)
        {
            for (int x = 0; x <= n - y; ++x)
            {
                subdivision.push_back(v0 + v_x * x + v_y * y);

//                std::cout << "(" << x << ", " << y << "), ";
                // first triangle
                // (x, y), (x+1, y), (x, y+1)
                // second (if possible)
                // (x+1, y), (x, y+1), (x+1, y+1)

            }
//            std::cout << "\n";
        }
    }

    // remove duplicated vertices
    subdivision = removeDuplicatedVertices(subdivision);

    triangles.clear();
    normals.clear();
    calculateTriangles(subdivision, triangles, normals);

    // project vertices on sphere and recalculate normals
    // triangles are the same and don't need to be estimated again
    // The triangle calculation algorithm doesn't work after the
    // projection because not all triangles are guaranteed to have
    // equaly lengthed edges.
    // A projection on a sphere works as follows:
    // For each vertex the vector that points from the center to
    // the vertex is extended so that is points on the boundary of
    // the sphere. This is done by multiplying the radius with the
    // normalized vector.

    // center vertex
    Vector center = calculateCenter(subdivision);


    // Projection
    for (Vector& v : subdivision)
    {
        Vector vBS = v - center;
        v = radius / vBS.norm() * vBS;
    }

    return Polygon2D(subdivision, triangles);
}

Polygon3D GeometricDataFactory::create3DSphere(
        double radius,
        int resolution,
        const MeshCriteria& meshCriteria)
{
    Polygon2D p2temp = create2DSphere(radius, resolution);
    Polygon3D p3 = createPolygon3DFromPolygon2D(p2temp, meshCriteria);
    return p3;
}

Polygon3D GeometricDataFactory::createPolygon3DFromPolygon2D(
        Polygon2D& p,
        const MeshCriteria& meshCriteria)
{
    Vectors verticesOut;
    Faces outerFacesOut;
    Faces facesOut;
    Cells cellsOut;
    MeshConverter::instance()->generateMesh(
                p.getPositions(),
                p.getTopology().retrieveFaces(),
                verticesOut,
                outerFacesOut,
                facesOut,
                cellsOut,
                meshCriteria);

    // check integrity

    // are there faces/ cells with duplicated indices?
    for (Face& f : facesOut)
    {
        if (f[0] == f[1] ||
            f[0] == f[2] ||
            f[1] == f[2])
        {
            std::cout << "face integrity error\n";
        }
    }

    for (Cell& c : cellsOut)
    {
        if (c[0] == c[1] ||
            c[0] == c[2] ||
            c[0] == c[3] ||
            c[1] == c[2] ||
            c[1] == c[3] ||
            c[2] == c[3])
        {
            std::cout << "cell integrity error\n";
        }

    }

    // are there vertices that are not part of a cell?
    std::set<unsigned int> verticesOfCells;
    for (Cell& c : cellsOut)
    {
        for (size_t i = 0; i < 4; ++i)
        {
            verticesOfCells.insert(c[i]);
        }
    }
    for (unsigned int i = 0; i < verticesOut.size(); ++i)
    {
        if (verticesOfCells.find(i) == verticesOfCells.end())
        {
            std::cout << "vertex integrity error\n";
        }
    }

    if (verticesOfCells.size() != verticesOut.size())
        std::cout << "vertex size missmatch\n";

    // check if points lie on top of each other? <- should be no issue
    // a cell is represented multiple times?
    std::set<Cell> alreadySeenCells;
    for (Cell& c : cellsOut)
    {
        if (alreadySeenCells.find(c) != alreadySeenCells.end())
            std::cout << "error: cell occurs multiple times!\n";
        alreadySeenCells.insert(c);
    }

    return Polygon3D(verticesOut,
                     TopologyFactory::createPolygon3DTopologyWithOuter(
                         facesOut, /*outerFacesOut,*/
                         cellsOut, verticesOut.size()));
//    return Polygon3D(verticesOut,
//                     TopologyFactory::createPolygon3DTopology(
//                         facesOut, outerFacesOut,
//                         cellsOut, verticesOut.size()));
}

GeometricDataFactory::GeometricDataFactory()
{
}

void GeometricDataFactory::calculateTriangles(
        const Vectors& vertices,
        Faces& trianglesOut,
        Vectors& normalsOut)
{

    // center vertex
    Vector center = calculateCenter(vertices);

    // Pseudo code:
    // Cheack for each vertex v
    //          for each pair of neighbored edges
    //              If there is an edge that shared the two vertices that are not v.
    //              If there is, these three edges make up a triangle.
    //              The triangle points away from the center vertex.

    // find all edges

    std::vector<std::tuple<size_t, size_t>> edges;
    for (size_t i = 0; i < vertices.size(); ++i)
    {
        const Vector& v = vertices[i];
        // distance, vertex
        std::vector<std::tuple<double, size_t>> potentialVertices;

        // save for each vertex the distance

        for (size_t j = 0; j < vertices.size(); ++j)
        {
            const Vector& v2 = vertices[j];

            double distance = (v - v2).norm();
            potentialVertices.push_back(std::make_tuple(distance, j));
        }

        // sort them by distance and remove first (which is v so distance 0.0)
        std::sort(potentialVertices.begin(), potentialVertices.end(),
                  [](const std::tuple<double, size_t>& t1,
                  const std::tuple<double, size_t>& t2)
        {
            return std::get<0>(t1) < std::get<0>(t2);
        });
        potentialVertices.erase(potentialVertices.begin());

        double distance = std::get<0>(potentialVertices[0]);
        for (const std::tuple<double, size_t>& t : potentialVertices)
        {
            if (std::abs(distance - std::get<0>(t)) > 1e-10)
                break;

            edges.push_back(std::make_tuple(i, std::get<1>(t)));
        }
    }

    // remove duplicated edges
    edges = removeDuplicatedEdges(edges);

    int nLoops1 = 0;
    int nLoops2 = 0;
    int nLoops3 = 0;
    // Find all triangles. A triangle consists of 3 edges that have
    // exactly 3 distinct vertices.
    Faces triangles;
    for (const std::tuple<size_t, size_t>& e1 : edges)
    {
        for (const std::tuple<size_t, size_t>& e2 : edges)
        {
            if (&e1 == &e2)
                continue;

            nLoops1++;
            std::vector<size_t> triangleVertices1;
            insertIfNotContains(e1, triangleVertices1);
            insertIfNotContains(e2, triangleVertices1);

            // Checks for edges that don't share a single vertex which
            // is most often the case.
            if (triangleVertices1.size() != 3)
                continue;

            for (const std::tuple<size_t, size_t>& e3 : edges)
            {
                if (&e1 == &e3 || &e2 == &e3)
                    continue;

                bool contains1 = false;
                for (size_t v : triangleVertices1)
                {
                    if (std::get<0>(e3) == v)
                        contains1 = true;
                }
                if (!contains1)
                    continue;

                bool contains2 = false;
                for (size_t v : triangleVertices1)
                {
                    if (std::get<1>(e3) == v)
                        contains2 = true;
                }

                if (!contains2)
                    continue;

                nLoops2++;
                std::vector<size_t> triangleVertices2 = triangleVertices1;
                insertIfNotContains(e3, triangleVertices2);

                if (triangleVertices2.size() != 3)
                    continue;

                nLoops3++;
                triangles.push_back({static_cast<unsigned int>(triangleVertices2[0]),
                                     static_cast<unsigned int>(triangleVertices2[1]),
                                     static_cast<unsigned int>(triangleVertices2[2])});
//                { findIndexOfVertex(triangleVertices2[0], vertices),
//                  findIndexOfVertex(triangleVertices2[1], vertices),
//                  findIndexOfVertex(triangleVertices2[2], vertices) });

            }
        }
    }
    std::cout << "nLoops = " << nLoops1 << ", " << nLoops2 << ", " << nLoops3 << "\n";

    triangles = removeDuplicatedTriangles(triangles);

    // calculate normals and reorder triangles according to normals
    for (const Face& f : triangles)
    {
        Vector e1 = vertices[f[0]] - vertices[f[1]];
        Vector e2 = vertices[f[0]] - vertices[f[2]];
        Vector normal = e1.cross(e2).normalized();
        Vector direction = vertices[f[0]] - center;

        if (normal.dot(direction) > 0)
        {
            trianglesOut.push_back(f);
            normalsOut.push_back(normal);
        }
        else
        {
            trianglesOut.push_back({ f[0], f[2], f[1] });
            normalsOut.push_back(-normal);
        }
    }


}

void GeometricDataFactory::insertIfNotContains(
        const std::tuple<size_t, size_t>& vertexPair,
        std::vector<size_t>& vertices)
{
    insertIfNotContains(std::get<0>(vertexPair), vertices);
    insertIfNotContains(std::get<1>(vertexPair), vertices);
}

void GeometricDataFactory::insertIfNotContains(
        size_t vertex,
        std::vector<size_t>& vertices)
{
    bool contains = false;
    for (size_t v : vertices)
    {
        if (v == vertex)
        {
            contains = true;
            break;
        }
    }
    if (!contains)
        vertices.push_back(vertex);
}

void GeometricDataFactory::insertIfNotContains(
        const Vector& vertex,
        std::vector<Vector>& vertices)
{
    bool contains = false;
    for (const Eigen::Vector& v : vertices)
    {
        if (v.isApprox(vertex, 1e-10))
        {
            contains = true;
            break;
        }
    }
    if (!contains)
        vertices.push_back(vertex);
}

unsigned int GeometricDataFactory::findIndexOfVertex(
        const Vector& v,
        const std::vector<Vector>& vertices)
{
    for (size_t i = 0; i < vertices.size(); ++i)
    {
        if (vertices[i].isApprox(v, 1e-10))
            return static_cast<unsigned int>(i);
    }
    return 0;
}

Vectors GeometricDataFactory::removeDuplicatedVertices(const Vectors& vertices)
{
    Vectors returnValue;
    for (const Vector& v : vertices)
    {
        insertIfNotContains(v, returnValue);
    }
    return returnValue;
}

std::vector<std::tuple<size_t, size_t>> GeometricDataFactory::removeDuplicatedEdges(
        const std::vector<std::tuple<size_t, size_t>>& edges)
{
    std::vector<std::tuple<size_t, size_t>> returnValue;

    for (const std::tuple<size_t, size_t>& e1 : edges)
    {
        auto it = std::find_if(returnValue.begin(), returnValue.end(), [e1](
                  const std::tuple<size_t, size_t>& e2)
        {
            return (std::get<0>(e1) == std::get<0>(e2) && std::get<1>(e1) == std::get<1>(e2)) ||
                    (std::get<0>(e1) == std::get<1>(e2) && std::get<1>(e1) == std::get<0>(e2));
        });
        if (it == returnValue.end())
        {
            returnValue.push_back(e1);
        }
    }

    return returnValue;
}

Faces GeometricDataFactory::removeDuplicatedTriangles(const Faces& triangles)
{
    Faces returnValue;

    for (const Face& e1 : triangles)
    {
        auto it = std::find_if(returnValue.begin(), returnValue.end(), [e1](
                  const Face& e2)
        {
           return (e1.at(0) == e2.at(0) && e1.at(1) == e2.at(1) && e1.at(2) == e2.at(2)) ||
                   (e1.at(0) == e2.at(0) && e1.at(1) == e2.at(2) && e1.at(2) == e2.at(1)) ||
                   (e1.at(0) == e2.at(1) && e1.at(1) == e2.at(0) && e1.at(2) == e2.at(2)) ||
                   (e1.at(0) == e2.at(1) && e1.at(1) == e2.at(2) && e1.at(2) == e2.at(0)) ||
                   (e1.at(0) == e2.at(2) && e1.at(1) == e2.at(0) && e1.at(2) == e2.at(1)) ||
                   (e1.at(0) == e2.at(2) && e1.at(1) == e2.at(1) && e1.at(2) == e2.at(0));
        });
        if (it == returnValue.end())
        {
            returnValue.push_back(e1);
        }
    }

    return returnValue;
}

Vector GeometricDataFactory::calculateCenter(const Vectors& /*vertices*/)
{
//    Vector center = Vector::Zero();
//    for (const Vector& v : vertices)
//    {
//        center += v;
//    }
//    center /= vertices.size();
//    return center;
    return Vector::Zero();
}
