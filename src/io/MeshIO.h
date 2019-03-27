#ifndef MESHIO_H
#define MESHIO_H

#include "data_structures/DataStructures.h"
#include <scene/data/geometric/Polygon2D.h>
#include <vector>

using namespace Eigen;

class MeshIO
{
public:
    MeshIO();

    static MeshIO* instance();

    // loads the content of an off file
    // writes positions, faces, bounding box
    // recalculates normals
    // sets positions and outer faces
    // calculates normals and bounding box
    // @param output_filename - filename of file where raw conversion is loaded to
    void loadOff(
            const char* filename,
            std::vector<Vector>& vertices,
            std::vector<Vector>& normals,
            std::vector<std::array<unsigned int, 3>>& triangles);

    Polygon2D* loadOff(const char* filename);

    // load SceneObject
    void loadRaw(
            std::string const& filename,
            std::vector<Vector>& vertices,
            std::vector<Vector>& normals,
            std::vector<std::array<unsigned int, 3>>& outer_triangles,
            std::vector<std::array<unsigned int, 3>>& triangles);

    // load SimulationObject
    void loadRaw(
            std::string const& filename,
            std::vector<Vector>& vertices,
            std::vector<Vector>& normals,
            std::vector<std::array<unsigned int, 3>>& outer_triangles,
            std::vector<std::array<unsigned int, 3>>& triangles,
            std::vector<std::array<unsigned int, 4>>& cells);

    // save SceneObject
    void saveRaw(
            std::string const& filename,
            std::vector<Vector> const& vertices,
            std::vector<Vector> const& normals,
            std::vector<std::array<unsigned int, 3>> const& outer_triangles,
            std::vector<std::array<unsigned int, 3>> const& triangles);

    // save SimulationObject
    void saveRaw(
            std::string const& filename,
            std::vector<Vector> const& vertices,
            std::vector<Vector> const& normals,
            std::vector<std::array<unsigned int, 3>> const& outer_triangles,
            std::vector<std::array<unsigned int, 3>> const& triangles,
            std::vector<std::array<unsigned int, 4>> const& cells);

private:
    template <typename T>
    void loadVector(std::ifstream& in, std::vector<T>& vector);

    template <typename T>
    void saveVector(std::ofstream& out, std::vector<T> const& vector);

    static MeshIO* m_instance;

};

#endif // MESHIO_H
