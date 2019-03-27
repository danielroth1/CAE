#ifndef MESH_CONVERTER_H
#define MESH_CONVERTER_H

#include <Eigen/Dense>
#include "data_structures/DataStructures.h"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Polyhedron_3<K> Polyhedron;

// This method provides methods from the CGAL library for the creationg
// of Polygon3D volumes from Polygon2D meshes. Polygon2Ds are made of
// triangles while Polygon3Ds are made of tetrahedrons. The MeshConverter
// can be used anytime it is desirable to simulate volumetric data while
// only a mesh is provided.
//
// Use generateMesh() to create Polygon3Ds from Polygon2Ds data.
class MeshConverter
{
private:
    MeshConverter();

public:
    static MeshConverter* instance() {
        return m_instance;
    }

    // Generates a 3D polygon from a given 2D polygon.
    // \param vertices - input vertices
    // \param facets - input triangles
    // \param vertices_out - output vertices
    // \param facets_out - output triangles
    // \param cells_out - output tetrahedrons
    // \param cellSize - a parameter that is used by CGAL.
    //              Defines the way the tetrahedrons are created.
    // \param cellRadiusEdgeRatio - a parameter that is used by CGAL.
    //              Defines the way the tetrahedrons are created.
    bool generateMesh(
            const Vectors& vertices,
            const Faces& facets,
            Vectors& vertices_out,
            Faces& outer_facets_out,
            Faces& facets_out,
            Cells& cells_out,
            double cellSize = 0.3, //0.08,
            double cellRadiusEdgeRatio = 30); //0.1);

    // The same as the other generateMesh() but uses a CGAL Polyhedron.
    // Calling createPolyhedron() and this method is equal to the other
    // generateMesh().
    bool generateMesh(
            Polyhedron& polyhedron,
            Vectors& vertices_out,
            Faces& outer_facets_out,
            Faces& facets_out,
            Cells& cells_out,
            double cellSize = 0.3,
            double cellRadiusEdgeRatio = 30);

    // Creates a CGAL Polyhedron from the given vertices and triangles.
    Polyhedron createPolyhedron(
            const Vectors& vertices,
            const Faces& facets);

    double getCellSize() const;
    double getCellRadiusEdgeRatio() const;

private:
    static MeshConverter* m_instance;

    double mCellSize;

    double mCellRadiusEdgeRatio;
};

#endif // MESH_CONVERTER_H
