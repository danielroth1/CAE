#ifndef MESHINTERPOLATORFEM_H
#define MESHINTERPOLATORFEM_H


#include "MeshInterpolator.h"
#include <data_structures/DataStructures.h>

class Polygon2D;
class Polygon3D;
class TopologyFace;
class VertexInterpolation;

// Maps each point of the target Polygon to a point in the source Polygon3D.
// Does so by calculating for each target vertex the barycentric coordinates
// within the source mesh.
// - If a vertex of the target mesh does not lie inside the source mesh, it will
// be mapped w.r.t. the closest element. These cases should be avoided because
// they can lead to strange mappings if the source mesh deformes too much.
// - Continous interpolation: It is also possible to get the barycentric
// coordinates of the source mesh for each point on the target mesh that is
// also described by barycentric coordinates (see calculateBary3()).
class MeshInterpolatorFEM : public MeshInterpolator
{
public:
    MeshInterpolatorFEM(const std::shared_ptr<Polygon3D>& source,
                        const std::shared_ptr<Polygon>& target);

    virtual ~MeshInterpolatorFEM() override;

    // Calculates the barycentric coordinates of the point at bary2 w.r.t.
    // the cell of the source mesh which contains the point.
    Eigen::Vector4d calculateBary3(ID targetTriangleId,
                                   const Eigen::Vector3d& bary2,
                                   ID& cellIdOut);

    // MeshInterpolator interface
public:
    // Calls geometricDataChanged() of target.
    virtual void solve() override;
    virtual void update() override;
    virtual Type getType() const override;
    virtual Eigen::Vector3d getSourcePosition(size_t targetId) const override;

private:

    // There is one per point of the high res mesh.
    struct VertexInterpolation
    {
        VertexInterpolation()
            : mAssigned(false)
        {

        }

        VertexInterpolation(Eigen::Vector4d weights,
                            std::size_t sourceCellIndex,
                            std::size_t targetVertexId)
            : mWeights(weights)
            , mSourceCellIndex(sourceCellIndex)
            , mTargetVertexId(targetVertexId)
            , mAssigned(true)
        {

        }

        // Baryzentric cooridnates w.r.t. a finite element of the low res mesh.
        Eigen::Vector4d mWeights;

        // The id of the corresponding cell on the low res mesh.
        std::size_t mSourceCellIndex;

        std::size_t mTargetVertexId;

        bool mAssigned;
    };

    // There is one triangle interpolation for each triangle of the target mesh.
    // It contains for each vertex the corresponding barycentric coordinates
    // for each tetrahedron the triangle is intersecting, e.g.:
    // A target triangle t has 2 vertices that are part of a cell A and one
    // vertex that is part of a cell B.
    // Then, its TriangleInterpolation will have 2 elementFunctions:
    // the first contains the barycentric coordinates of the three vertices
    // w.r.t. A
    // the second contains the barycentric coordinates of the three certices
    // w.r.t. B
    //
    struct TriangleInterpolation
    {
        TriangleInterpolation()
        {

        }

        std::vector<std::array<Eigen::Vector4d, 3>> elementFunctions;
        std::vector<ID> cellIds;
    };

    Eigen::Vector3d interpolate(const VertexInterpolation& vi) const;

    // Fills for all triangles the data necessary for a continous calculation
    // of barycentric coordinates from the target to the source mesh.
    // It enables the use of the function calculateBary3().
    // Fills mTriangleInterpolations.
    // Note: Currently, tetrahedrons that are intersecting a triangle but don't
    // contain any of the triangles vertices, will be undetected. In these
    // cases, it can happen, that calculateBary3() will return another element
    // with barycentric coordinates where one is above or below zero.
    // Adding these cases requires the calulcation of trianle-tetrahedron
    // collision detection which can be complicated.
    void solveContinuous();

    static Eigen::Vector4d calculateBaryzentricCoordinates(
            const std::shared_ptr<Polygon3D>& source,
            const Cell& c,
            const Eigen::Vector& p);

    // Projects the point p onto the triangle plane of f and calculates the
    // barycentric coordinates. The barycentric coordinates are truncated
    // so that all of them are 0 < a < 1 and a1 + a2 + a3 + a4 = 1.
    static Eigen::Vector3d calculateBaryzentricCoordinates(
            const std::shared_ptr<Polygon3D>& source,
            const TopologyFace& f,
            const Eigen::Vector& p);

    std::shared_ptr<Polygon3D> mSource3;

    bool mSolved;

    // Baryzentric cooridnates w.r.t. the low res mesh.
    // There is one per point of the high res mesh.
    std::vector<VertexInterpolation> mInterpolations;

    // For continuous mapping of barycentric coordinates. Is filled in
    // solveConitnous().
    std::vector<TriangleInterpolation> mTriangleInterpolations;
};

#endif // MESHINTERPOLATORFEM_H
