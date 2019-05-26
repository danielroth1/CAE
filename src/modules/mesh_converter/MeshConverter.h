#ifndef MESH_CONVERTER_H
#define MESH_CONVERTER_H

#include <Eigen/Dense>
#include "data_structures/DataStructures.h"


class MeshCriteria;

// This method provides methods from the CGAL library for the creationg
// of Polygon3D volumes from Polygon2D meshes. Polygon2Ds are made of
// triangles while Polygon3Ds are made of tetrahedrons. The MeshConverter
// can be used anytime it is desirable to simulate volumetric data while
// only a mesh is provided.
//
// Many of the methods of this class are implemented outside of the class definition
// directly in the .cpp. This is done to speed up compilation by avoiding cgal includes
// Since CGAL is a highly templated library, there is no possibility to use
// forward declarations.
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

    bool generateMesh(
            const Vectors& vertices,
            const Faces& facets,
            Vectors& vertices_out,
            Faces& outer_facets_out,
            Faces& facets_out,
            Cells& cells_out,
            const MeshCriteria& criteria);

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
            double facetAngle = 0,
            double facetSize = 0,
            double facetDistance = 0,
            double cellSize = 0,
            double cellRadiusEdgeRatio = 0);

private:
    static MeshConverter* m_instance;
};

#endif // MESH_CONVERTER_H
