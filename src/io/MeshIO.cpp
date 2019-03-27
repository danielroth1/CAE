#include "MeshIO.h"

#include <iostream>
#include <fstream>
#include <QDebug>
#include <iomanip>

MeshIO* MeshIO::m_instance = new MeshIO();

MeshIO::MeshIO()
{

}

MeshIO *MeshIO::instance()
{
    return m_instance;
}

void MeshIO::loadOff(
        const char *filename,
        std::vector<Vector> &vertices,
        std::vector<Vector> &normals,
        std::vector<std::array<unsigned int, 3> > &triangles)
{
    qDebug() << "load off file: " << filename;
    std::ifstream in(filename);
    if (!in.is_open())
    {
        std::cout << "loadOFF: can not open " << filename << std::endl;
        return;
    }
    const int MAX = 256;
    char s[MAX];
    in >> std::setw(MAX) >> s;
    // differentiate between OFF (vertices only) and NOFF (vertices and normals)
    bool noff = false;

    if (s[0] == 'O' && s[1] == 'F' && s[2] == 'F')
        ;
    else if (s[0] == 'N' && s[1] == 'O' && s[2] == 'F' && s[3] == 'F')
        noff = true;
    else
        return;

    // get number of vertices nv, faces nf and edges ne
    int nv, nf, ne;
    in >> std::setw(MAX) >> nv;
    in >> std::setw(MAX) >> nf;
    in >> std::setw(MAX) >> ne;
    if (nv <= 0 || nf <= 0)
        return;

    // read vertices
    vertices.resize(nv);
    if (noff)
        normals.resize(nv);
    for (int i = 0; i < nv; ++i)
    {
        in >> std::setw(MAX) >> vertices[i][0];
        in >> std::setw(MAX) >> vertices[i][1];
        in >> std::setw(MAX) >> vertices[i][2];
        if (noff)
        {
            in >> std::setw(MAX) >> normals[i][0];
            in >> std::setw(MAX) >> normals[i][1];
            in >> std::setw(MAX) >> normals[i][2];
        }
    }
    // read triangles
    triangles.resize(nf);
    for (int i = 0; i < nf; ++i) {
        int three;
        in >> std::setw(MAX) >> three;
        in >> std::setw(MAX) >> triangles[i][0];
        in >> std::setw(MAX) >> triangles[i][1];
        in >> std::setw(MAX) >> triangles[i][2];
    }

//    // TODO: correct the triangle indices
//    for (int i = 0; i < triangles.size(); ++i) {
//        std::array<unsigned int, 3>& cell = triangles[i];

//        Vector3d r_1 = vertices[cell[1]] - vertices[cell[0]];
//        Vector3d r_2 = vertices[cell[2]] - vertices[cell[0]];
//        Vector3d r_3 = vertices[cell[3]] - vertices[cell[0]];

//        if (r_1.cross(r_2).normalized().dot(r_3) < 0) { // TODO
//            unsigned int temp = cell[1];
//            cell[1] = cell[2];
//            cell[2] = temp;
//            std::cout << "ERROR: vertex not consistent!\n";
//        }
//    }

    // close ifstream
    in.close();
    // calculate normals if not given
//    if (!noff)
//        calculateNormals();
}

Polygon2D* MeshIO::loadOff(const char* filename)
{
    Vectors positions;
    Vectors normals;
    Faces faces;

    loadOff(filename, positions, normals, faces);

    if (!normals.empty())
        return new Polygon2D(positions, normals, faces);

    return new Polygon2D(positions, faces);

}

void MeshIO::loadRaw(
        const std::string &filename,
        std::vector<Vector>& vertices,
        std::vector<Vector>& normals,
        std::vector<std::array<unsigned int, 3> >& outer_triangles,
        std::vector<std::array<unsigned int, 3> >& triangles)
{
    std::ifstream in(filename, std::ios::in | std::ios::binary);

    if (in.fail())
    {
        std::ostringstream error;
        error << "Unable to open " << filename << ".\n";
        throw std::runtime_error(error.str().c_str());
    }

    // Vertices
    unsigned int nVertices;
    in.read(reinterpret_cast<char*>(&nVertices), sizeof(unsigned int));
    vertices.resize(nVertices);
    for (unsigned int i = 0; i < nVertices; ++i)
    {
        in.read(reinterpret_cast<char*>(vertices[i].data()), 3 * sizeof(double));
    }

    // Normals
    unsigned int nNormals;
    in.read(reinterpret_cast<char*>(&nNormals), sizeof(unsigned int));
    normals.resize(nNormals);
    for (unsigned int i = 0; i < nNormals; ++i)
    {
        in.read(reinterpret_cast<char*>(normals[i].data()), 3 * sizeof(double));
    }

    // Outer Triangles
    unsigned int nOuterTriangles;
    in.read(reinterpret_cast<char*>(&nOuterTriangles), sizeof(unsigned int));
    outer_triangles.resize(nOuterTriangles);
    for (unsigned int i = 0; i < nOuterTriangles; ++i)
    {
        in.read(reinterpret_cast<char*>(outer_triangles[i].data()), 3 * sizeof(unsigned int));
    }

    // Triangles
    unsigned int nTriangles;
    in.read(reinterpret_cast<char*>(&nTriangles), sizeof(unsigned int));
    triangles.resize(nTriangles);
    for (unsigned int i = 0; i < nTriangles; ++i)
    {
        in.read(reinterpret_cast<char*>(triangles[i].data()), 3 * sizeof(unsigned int));
    }
}

void MeshIO::loadRaw(
        const std::string &filename,
        std::vector<Vector>& vertices,
        std::vector<Vector>& normals,
        std::vector<std::array<unsigned int, 3> >& outer_triangles,
        std::vector<std::array<unsigned int, 3> >& triangles,
        std::vector<std::array<unsigned int, 4> >& cells)
{
    std::ifstream in(filename, std::ios::in | std::ios::binary);

    if (in.fail())
    {
        std::ostringstream error;
        error << "Unable to open " << filename << ".\n";
        throw std::runtime_error(error.str().c_str());
    }

    // Vertices
    unsigned int nVertices;
    in.read(reinterpret_cast<char*>(&nVertices), sizeof(unsigned int));
    vertices.resize(nVertices);
    for (unsigned int i = 0; i < nVertices; ++i)
    {
        in.read(reinterpret_cast<char*>(vertices[i].data()), 3 * sizeof(double));
    }

    // Normals
    unsigned int nNormals;
    in.read(reinterpret_cast<char*>(&nNormals), sizeof(unsigned int));
    normals.resize(nNormals);
    for (unsigned int i = 0; i < nNormals; ++i)
    {
        in.read(reinterpret_cast<char*>(normals[i].data()), 3 * sizeof(double));
    }

    // Outer Triangles
    unsigned int nOuterTriangles;
    in.read(reinterpret_cast<char*>(&nOuterTriangles), sizeof(unsigned int));
    outer_triangles.resize(nOuterTriangles);
    for (unsigned int i = 0; i < nOuterTriangles; ++i)
    {
        in.read(reinterpret_cast<char*>(outer_triangles[i].data()), 3 * sizeof(unsigned int));
    }

    // Triangles
    unsigned int nTriangles;
    in.read(reinterpret_cast<char*>(&nTriangles), sizeof(unsigned int));
    triangles.resize(nTriangles);
    for (unsigned int i = 0; i < nTriangles; ++i)
    {
        in.read(reinterpret_cast<char*>(triangles[i].data()), 3 * sizeof(unsigned int));
    }

    // Cells
    unsigned int nCells;
    in.read(reinterpret_cast<char*>(&nCells), sizeof(unsigned int));
    cells.resize(nCells);
    for (unsigned int i = 0; i < nCells; ++i)
    {
        in.read(reinterpret_cast<char*>(cells[i].data()), 4 * sizeof(unsigned int));
    }
}

void MeshIO::saveRaw(
        const std::string &filename,
        const std::vector<Vector>& vertices,
        const std::vector<Vector> &normals,
        const std::vector<std::array<unsigned int, 3> > &outer_triangles,
        const std::vector<std::array<unsigned int, 3> > &triangles)
{
    std::ofstream out(filename, std::ios::out | std::ios::binary);

    if (out.fail())
    {
        std::ostringstream error;
        error << "Unable to open " << filename << ".\n";
        throw std::runtime_error(error.str().c_str());
    }

    // Vertices
    unsigned int nVertices = static_cast<unsigned int>(vertices.size());
    out.write(reinterpret_cast<char*>(nVertices), sizeof(unsigned int));
    for (unsigned int i = 0; i < nVertices; ++i)
    {
        out.write(reinterpret_cast<const char*>(vertices[i].data()), 3 * sizeof(double));
    }

    // Normals
    unsigned int nNormals = static_cast<unsigned int>(normals.size());
    out.write(reinterpret_cast<char*>(nNormals), sizeof(unsigned int));
    for (unsigned int i = 0; i < nNormals; ++i)
    {
        out.write(reinterpret_cast<const char*>(normals[i].data()), 3 * sizeof(double));
    }

    // Outer Triangles
    unsigned int nOuterTriangles = static_cast<unsigned int>(outer_triangles.size());
    out.write(reinterpret_cast<char*>(nOuterTriangles), sizeof(unsigned int));
    for (unsigned int i = 0; i < nOuterTriangles; ++i)
    {
        out.write(reinterpret_cast<const char*>(outer_triangles[i].data()), 3 * sizeof(unsigned int));
    }

    // Triangles
    unsigned int nTriangles = static_cast<unsigned int>(triangles.size());
    out.write(reinterpret_cast<char*>(nTriangles), sizeof(unsigned int));
    for (unsigned int i = 0; i < nTriangles; ++i)
    {
        out.write(reinterpret_cast<const char*>(triangles[i].data()), 3 * sizeof(unsigned int));
    }
}

void MeshIO::saveRaw(
        const std::string &filename,
        const std::vector<Vector>& vertices,
        const std::vector<Vector> &normals,
        const std::vector<std::array<unsigned int, 3> > &outer_triangles,
        const std::vector<std::array<unsigned int, 3> > &triangles,
        const std::vector<std::array<unsigned int, 4> > &cells)
{
    std::ofstream out(filename, std::ios::out | std::ios::binary);

    if (out.fail())
    {
        std::ostringstream error;
        error << "Unable to open " << filename << ".\n";
        throw std::runtime_error(error.str().c_str());
    }

    // Vertices
    unsigned int nVertices = static_cast<unsigned int>(vertices.size());
    out.write(reinterpret_cast<char*>(nVertices), sizeof(unsigned int));
    for (unsigned int i = 0; i < nVertices; ++i)
    {
        out.write(reinterpret_cast<const char*>(vertices[i].data()), 3 * sizeof(double));
    }

    // Normals
    unsigned int nNormals = static_cast<unsigned int>(normals.size());
    out.write(reinterpret_cast<char*>(nNormals), sizeof(unsigned int));
    for (unsigned int i = 0; i < nNormals; ++i)
    {
        out.write(reinterpret_cast<const char*>(normals[i].data()), 3 * sizeof(double));
    }

    // Outer Triangles
    unsigned int nOuterTriangles = static_cast<unsigned int>(outer_triangles.size());
    out.write(reinterpret_cast<char*>(nOuterTriangles), sizeof(unsigned int));
    for (unsigned int i = 0; i < nOuterTriangles; ++i)
    {
        out.write(reinterpret_cast<const char*>(outer_triangles[i].data()), 3 * sizeof(unsigned int));
    }

    // Triangles
    unsigned int nTriangles = static_cast<unsigned int>(triangles.size());
    out.write(reinterpret_cast<char*>(nTriangles), sizeof(unsigned int));
    for (unsigned int i = 0; i < nTriangles; ++i)
    {
        out.write(reinterpret_cast<const char*>(triangles[i].data()), 3 * sizeof(unsigned int));
    }

    // Cells
    unsigned int nCells = static_cast<unsigned int>(cells.size());
    out.write(reinterpret_cast<char*>(nCells), sizeof(unsigned int));
    for (unsigned int i = 0; i < nCells; ++i)
    {
        out.write(reinterpret_cast<const char*>(cells[i].data()), 4 * sizeof(unsigned int));
    }
}


template<typename T>
void MeshIO::loadVector(std::ifstream &in, std::vector<T>& vector)
{
    unsigned int nElements = static_cast<unsigned int>(vector.size());
    in.read(reinterpret_cast<char*>(nElements), sizeof(unsigned int));
    for (int i = 0; i < vector.size(); ++i)
    {
        in.read(reinterpret_cast<char*>(vector[i]), sizeof(T));
    }
}


template<typename T>
void MeshIO::saveVector(std::ofstream &out, const std::vector<T>& vector)
{
    unsigned int nElements = static_cast<unsigned int>(vector.size());
    out.write(reinterpret_cast<char*>(nElements), sizeof(unsigned int));
    for (int i = 0; i < vector.size(); ++i)
    {
        out.write(reinterpret_cast<char*>(vector[i]), sizeof(T));
    }
}
