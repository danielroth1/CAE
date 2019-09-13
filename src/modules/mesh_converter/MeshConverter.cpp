#include "MeshConverter.h"
#include "MeshCriteria.h"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/make_mesh_3.h>
#include <CGAL/Mesh_3/config.h>
#include <CGAL/Mesh_complex_3_in_triangulation_3.h>
#include <CGAL/Mesh_criteria_3.h>
#include <CGAL/Mesh_triangulation_3.h>
#include <CGAL/Mesh_polyhedron_3.h>
#include <CGAL/Polyhedral_mesh_domain_3.h>
#include <CGAL/Polyhedral_mesh_domain_with_features_3.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/refine_mesh_3.h>
#include <CGAL/utility.h>

// IO
#include <iostream>
#include <fstream>

// Boost
#include <boost/algorithm/string/predicate.hpp>
#include <boost/lexical_cast.hpp>

// Eigen
#include <Eigen/Dense>

// std
#include <set>
#include <vector>

// CGAL feature detection
#include <CGAL/Polygon_mesh_processing/detect_features.h>

// concurrency tags
#ifdef CGAL_CONCURRENT_MESH_3
typedef CGAL::Parallel_tag Concurrency_tag;
#else
typedef CGAL::Sequential_tag Concurrency_tag;
#endif


// Polyhedron
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Mesh_polyhedron_3<K>::type Polyhedron;

// Domain
typedef CGAL::Polyhedral_mesh_domain_with_features_3<K> Mesh_domain_features;
typedef CGAL::Polyhedral_mesh_domain_3<Polyhedron, K> Mesh_domain;

// Triangulation
typedef CGAL::Mesh_triangulation_3<Mesh_domain, CGAL::Default, Concurrency_tag>::type Tr;
typedef CGAL::Mesh_triangulation_3<Mesh_domain_features, CGAL::Default, Concurrency_tag>::type Tr_features;

typedef CGAL::Mesh_complex_3_in_triangulation_3<Tr> C3t3;
typedef CGAL::Mesh_complex_3_in_triangulation_3<Tr> C3t3_features;

// Criteria
typedef CGAL::Mesh_criteria_3<Tr> Mesh_criteria;

typedef Polyhedron::HalfedgeDS HalfedgeDS;


MeshConverter* MeshConverter::m_instance = new MeshConverter();

MeshConverter::MeshConverter()
{

}

// Helper function
template<typename T, unsigned int n>
void add_if_not_contains(std::vector<std::array<T, n>>& v, std::array<T, n> e)
{
    for (unsigned int i = 0; i < v.size(); ++i)
    {
        bool equal = true;
        for (unsigned int j = 0; j < n; ++j)
        {
            if (std::abs(v[i][j] - e[j]) > 0.0001)
                equal = false;
        }
        if (equal)
            return;
    }
    v.push_back(e);
}


template<typename Polyhedron>
void reset_sharp_edges(Polyhedron* pMesh)
{
    typename boost::property_map<Polyhedron, CGAL::edge_is_feature_t>::type if_pm =
            get(CGAL::edge_is_feature, *pMesh);
    for(typename boost::graph_traits<Polyhedron>::edge_descriptor ed : edges(*pMesh))
    {
        put(if_pm,ed,false);
    }
}
template<typename Polyhedron>
void detect_sharp_edges(Polyhedron* pMesh, const double angle)
{
    reset_sharp_edges(pMesh);

    // Detect edges in current polyhedron
    typename boost::property_map<Polyhedron, CGAL::edge_is_feature_t>::type eif =
            get(CGAL::edge_is_feature, *pMesh);

    CGAL::Polygon_mesh_processing::detect_sharp_edges(*pMesh, angle, eif);
}

// Creates a CGAL Polyhedron from the given vertices and triangles.
Polyhedron createPolyhedron(
        const Vectors& vertices,
        const Faces& facets)
{
    Polyhedron p;

    CGAL::Polyhedron_incremental_builder_3<HalfedgeDS> builder( p.hds(), true);
    typedef typename HalfedgeDS::Vertex   Vertex;
    typedef typename Vertex::Point Point;

    builder.begin_surface(vertices.size(), facets.size(), 0);

    for (const Eigen::Vector3d& v : vertices)
    {
        builder.add_vertex(Point(v(0), v(1), v(2)));
    }

    for (const std::array<unsigned int, 3>& f : facets)
    {
        builder.begin_facet();
        builder.add_vertex_to_facet(f[0]);
        builder.add_vertex_to_facet(f[1]);
        builder.add_vertex_to_facet(f[2]);
        builder.end_facet();
    }

    builder.end_surface();

    return p;
}

template <typename Mesh_domain>
bool generateMeshFromCGALPolyhedron(
        Mesh_domain& domain,
        Vectors& vertices_out,
        Faces& outer_facets_out,
        Faces& facets_out,
        Cells& cells_out,
        const MeshCriteria& meshCriteria)
{

    // Triangulation
    typedef typename CGAL::Mesh_triangulation_3<Mesh_domain, CGAL::Default, Concurrency_tag>::type MeshTr;
    typedef CGAL::Mesh_complex_3_in_triangulation_3<MeshTr> C3t3;
    typedef CGAL::Mesh_criteria_3<MeshTr> Mesh_criteria;

    Mesh_criteria criteria(
                CGAL::parameters::cell_radius_edge_ratio=meshCriteria.getCellRadiusEdgeRatio(),
                CGAL::parameters::cell_size=meshCriteria.getCellSize(),
                CGAL::parameters::facet_angle=meshCriteria.getFacetAngle(),
                CGAL::parameters::facet_size=meshCriteria.getFacetSize(),
                CGAL::parameters::facet_distance=meshCriteria.getFaceDistance());

//    Mesh_criteria criteria;

    // Mesh generation
    C3t3 c3t3 = CGAL::make_mesh_3<C3t3>(
                domain, criteria,
                CGAL::parameters::no_perturb(),
                CGAL::parameters::no_exude());


    // CONVERT FROM CGAL TO vectors
    typedef typename C3t3::Triangulation Tr;
    typedef typename C3t3::Facets_in_complex_iterator Facet_iterator;
    typedef typename C3t3::Cells_in_complex_iterator Cell_iterator;

    typedef typename Tr::Finite_vertices_iterator Finite_vertices_iterator;
    typedef typename Tr::Vertex_handle Vertex_handle;
    typedef typename Tr::Point Point_3;

    const Tr& tr = c3t3.triangulation();

    std::cout << "\nAfter Triangulation: \n" << "Vertices: "
            << tr.number_of_vertices() << "\n"
            << "Facets/Triangles: " << tr.number_of_facets() << "\n"
            << "Cells/Tetrahedra: " << tr.number_of_cells() << "\n";

    //-------------------------------------------------------
    // Vertices
    //-------------------------------------------------------

    boost::unordered_map<Vertex_handle, int> V;
    int inum = 0;
    for (Finite_vertices_iterator vit = tr.finite_vertices_begin();
            vit != tr.finite_vertices_end(); ++vit) {
        V[vit] = inum++;
        Point_3 p = vit->point();
        vertices_out.push_back(Eigen::Vector3d((float)CGAL::to_double(p.x()),
                (float)CGAL::to_double(p.y()), (float)CGAL::to_double(p.z())));
    }

    //-------------------------------------------------------
    // Facets
    //-------------------------------------------------------
    typename C3t3::size_type number_of_triangles =
            c3t3.number_of_facets_in_complex();

    for (Facet_iterator fit = c3t3.facets_in_complex_begin();
            fit != c3t3.facets_in_complex_end(); ++fit) {
        typename C3t3::Subdomain_index cell_sd=c3t3.subdomain_index(fit->first);
        typename C3t3::Subdomain_index opp_sd=c3t3.subdomain_index(fit->first->neighbor(fit->second));

        int j = -1;
        std::array<unsigned int, 3> vertex;

        for (int i = 0; i < 4; i++) {
            if (i != fit->second) {
                const Vertex_handle& vh = (*fit).first->vertex(i);
                vertex[++j] = V[vh];
            }
        }

        //facets.push_back(vertex);
        // Only for outer Facets true (that one that lie on the boundary)
        if (!(cell_sd != 0 && opp_sd != 0))
            outer_facets_out.push_back(vertex);
    }

    //-------------------------------------------------------
    // Tetrahedra
    //-------------------------------------------------------

    auto sortedFace = [](std::array<unsigned int, 3> f)
    {
        std::sort(f.begin(), f.end());
        return f;
    };

    std::set<std::array<unsigned int, 3>> addedSortedFacets;
    auto addFaceIfNotContains = [&sortedFace, &facets_out, &addedSortedFacets](
            std::array<unsigned int, 3> f)
    {
        std::array<unsigned int, 3> sortedF = sortedFace(f);
        if (addedSortedFacets.find(sortedF) == addedSortedFacets.end())
        {
            addedSortedFacets.insert(sortedF);
            facets_out.push_back(f);
        }
    };

    for (Cell_iterator cit = c3t3.cells_in_complex_begin();
            cit != c3t3.cells_in_complex_end(); ++cit)
    {
        std::array<unsigned int, 4> f;
        for (unsigned int i = 0; i < 4; i++)
            f[i] = static_cast<unsigned int>(V[cit->vertex(static_cast<int>(i))]);

        cells_out.push_back(f);

        addFaceIfNotContains({f[0], f[1], f[2]});
        addFaceIfNotContains({f[0], f[1], f[3]});
        addFaceIfNotContains({f[0], f[2], f[3]});
        addFaceIfNotContains({f[1], f[2], f[3]});
    }

    // correct the triangle indices
    // The 3 vertex indices of a triangles must be ordered in a way
    // that the cross product of the first two vertices:
    // v1.cross(v2) points on the outside.
    for (size_t i = 0; i < outer_facets_out.size(); ++i) {
        std::array<unsigned int, 3>& facet = outer_facets_out[i];

        // =====================================================================
        // Disclaimer:
        // This code part is responsible for fixing the vertex index order of
        // each outer triangle so that the normal that points outside the
        // polygon can be calculated in a consistent way. The same is already
        // done in the constructor of Polygon3D so this part may be removed.
        // It is left here for the case that one doesn't want to create a
        // Polygon3D afterwards. It could be removed for the future to slightly
        // improve the performance.

        // find the outer facet (tetrahedron) that contains this triangle
        // note, that since this is an outer triangle, there is only a single tetrahedron
        // that containts it.
        bool found = false;
        unsigned int other_vertex_id = 0;
        for (size_t j = 0; j < cells_out.size(); ++j)
        {
            Cell& c = cells_out[j];
            if (std::find(c.begin(), c.end(), facet[0]) != c.end() &&
                std::find(c.begin(), c.end(), facet[1]) != c.end()&&
                std::find(c.begin(), c.end(), facet[2]) != c.end())
            {
                found = true;

                // find the other vertex of the cell that is not part of the triangle
                for (size_t k = 0; k < 4; ++k)
                {
                    if (std::find(facet.begin(), facet.end(), c[k]) == facet.end())
                    {
                        other_vertex_id = c[k];
                        break;
                    }
                }

                break;
            }
        }

        if (!found)
        {
            std::cout << "Did not find cell for outer triangle.\n";
        }
        else
        {
            Eigen::Vector3d r_1 = vertices_out[facet[1]] - vertices_out[facet[0]];
            Eigen::Vector3d r_2 = vertices_out[facet[2]] - vertices_out[facet[0]];
            Eigen::Vector3d r_3 = vertices_out[other_vertex_id] - vertices_out[facet[0]];
//            std::cout << "other_vertex_id = " << other_vertex_id << "\n";

            if (r_1.cross(r_2).normalized().dot(r_3) > 0)
            {
                unsigned int temp = facet[1];
                facet[1] = facet[2];
                facet[2] = temp;
            }
        }

        // =====================================================================

        // is this face part of all faces?
        bool found2 = false;
        for (size_t j = 0; j < facets_out.size(); ++j)
        {
            if (sortedFace(facets_out[j]) == sortedFace(facet))
            {
                found2 = true;
                break;
            }
        }
        if (!found2)
        {
            std::cout << "Outer face is not part of all faces.\n";
        }
    }

    std::cout << "\nIn Output File: \n" << "Vertices: "
            << vertices_out.size() << "\n"
            << "Facets/Triangles: " << facets_out.size() << "\n"
            << "Outer Facets/Triangles: " << outer_facets_out.size() << "\n"
            << "Cells/Tethraedra: " << cells_out.size() << "\n";

    return true;
}


// The same as the other generateMesh() but uses a CGAL Polyhedron.
// Calling createPolyhedron() and this method is equal to the other
// generateMesh().
bool generateMeshFromCGALPolyhedron(
        Polyhedron& p,
        Vectors& vertices_out,
        Faces& outer_facets_out,
        Faces& facets_out,
        Cells& cells_out,
        const MeshCriteria& meshCriteria)
{
    if (meshCriteria.isEdgesAsFeatures())
    {
        typedef CGAL::Polyhedral_mesh_domain_with_features_3<K> Mesh_domain;

        // Create domain
        Mesh_domain domain(p, p);
        domain.detect_features(meshCriteria.getMinFeatureEdgeAngleDeg());

        return generateMeshFromCGALPolyhedron(
                    domain, vertices_out, outer_facets_out, facets_out, cells_out,
                    meshCriteria);
    }
    else
    {
        typedef CGAL::Polyhedral_mesh_domain_3<Polyhedron, K> Mesh_domain;

        // Create domain
        Mesh_domain domain(p);

        return generateMeshFromCGALPolyhedron(
                    domain, vertices_out, outer_facets_out, facets_out, cells_out,
                    meshCriteria);
    }

}


bool MeshConverter::generateMesh(
        const Vectors& vertices,
        const Faces& facets,
        Vectors& vertices_out,
        Faces& outer_facets_out,
        Faces& facets_out,
        Cells& cells_out,
        const MeshCriteria& meshCriteria)
{
    std::cout << "Input: \n "
              << "Vertices: " << vertices.size() << "\n"
              << "Facets/Triangles: " << facets.size() << "\n";

    vertices_out.clear();
    outer_facets_out.clear();
    facets_out.clear();
    cells_out.clear();

    Polyhedron p = createPolyhedron(vertices, facets);

    std::cout << "\nAfter Conversion to Polyhedron: \n"
              << "Vertices: " << p.size_of_vertices() << "\n"
              << "Facets/Triangles: " << p.size_of_facets() << "\n";

    return generateMeshFromCGALPolyhedron(
                p, vertices_out, outer_facets_out, facets_out, cells_out, meshCriteria);


}
