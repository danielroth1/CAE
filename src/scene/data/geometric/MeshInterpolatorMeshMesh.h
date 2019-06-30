#ifndef MESHINTERPOLATORMESHMESH_H
#define MESHINTERPOLATORMESHMESH_H

#include "MeshInterpolator.h"
#include <Eigen/Core>

class Polygon2DAccessor;
class TopologyFace;

class MeshInterpolatorMeshMesh : public MeshInterpolator
{
public:
    MeshInterpolatorMeshMesh(const std::shared_ptr<Polygon>& source,
                             const std::shared_ptr<Polygon>& target);

    virtual void update() override;

    // Returns the id of the source vertex w.r.t. the target vertex with the
    // given id.
    virtual size_t getSourceFaceId(size_t targetId) const override;

    // Returns the position of the source vertex that correspondons to the
    // target vertex with the given id.
    virtual Eigen::Vector3d getSourcePosition(size_t targetId) const override;

private:

    // There is one per point of the high res mesh.
    struct VertexInterpolation
    {
        VertexInterpolation()
            : mAssigned(false)
        {

        }

        VertexInterpolation(const Eigen::Vector3d& weights,
                            double distance,
                            std::size_t sourceFaceId,
                            std::size_t targetVertexId)
            : mWeights(weights)
            , mDistance(distance)
            , mSourceFaceId(sourceFaceId)
            , mTargetVertexId(targetVertexId)
            , mAssigned(true)
        {

        }

        // Baryzentric cooridnates w.r.t. the low res mesh.
        Eigen::Vector3d mWeights;

        // Distance of the point on the low res mesh to the one on the high res mesh.
        double mDistance;

        // The id of the corresponding vertex on the low res mesh.
        std::size_t mSourceFaceId;

        std::size_t mTargetVertexId;

        bool mAssigned;
    };

    struct GradientDescentParameters
    {
        GradientDescentParameters(
                size_t _maxIterations = 2000,
                double _accuracy = 1e-5)
            : maxIterations(_maxIterations)
            , accuracy(_accuracy)
        {

        }

        size_t maxIterations;

        // Maximum accepted error. The error is the norm of the gradient.
        double accuracy;

        static const GradientDescentParameters& defaultValues()
        {
            return DEFAULT_GD_PARAMS;
        }

    private:
        static GradientDescentParameters DEFAULT_GD_PARAMS;
    };

    struct LineSearchParameters
    {
        LineSearchParameters(
                size_t _maxIterations = 10,
                double _c = 0.1,
                double _tau = 0.5)
            : maxIterations(_maxIterations)
            , c(_c)
            , tau(_tau)
        {

        }

        // Maximum number of line search iterations. This value doesn't need
        // to be too high because with each iteration the step size is reduced
        // the factor tau, so e.g. if in iteration 5 and tau = 0.5, the step
        // size would be tau^8 = 0.5^8 = very low...
        // By using higher maximum iterations and lower values for tau, the
        // search space can be distretized higher.
        size_t maxIterations;

        // acceptance value
        // The higher this value, the harder it is for a value to be accepted.
        // If this value is too high, the line search is more likely to
        // search for a better suited value until the max iteration count is
        // reached. If the value is too low, worse step sizes could be accepted,
        // terminating the line search too early
        double c = 0.1;

        // Factor with which the step size is reduced in each line search step.
        double tau = 0.5;

        static const LineSearchParameters& defaultValues()
        {
            return DEFAULT_LS_PARAMS;
        }

    private:
        static LineSearchParameters DEFAULT_LS_PARAMS;
    };

    void init();

    Eigen::Vector4d calculateWeightsSimpleGD(
            Eigen::Vector3d p,
            Eigen::Vector3d v[3],
            Eigen::Vector3d n[3],
            Eigen::Vector4d weights);

    Eigen::Vector3d calculateWeightsImprovedGD(
            Eigen::Vector3d p,
            Eigen::Vector3d v[3],
            Eigen::Vector3d n[3],
            Eigen::Vector4d weights,
            bool& converged,
            const GradientDescentParameters& gdParams,
            const LineSearchParameters& lsParams,
            bool printStatistics = false);

    Eigen::Vector3d interpolate(
            const VertexInterpolation& interpolation);

    // Baryzentric cooridnates w.r.t. the low res mesh.
    // There is one per point of the high res mesh.
    std::vector<VertexInterpolation> mInterpolations;
};

#endif // MESHINTERPOLATORMESHMESH_H
