#ifndef MESHINTERPOLATORMESHMESH_H
#define MESHINTERPOLATORMESHMESH_H

#include "MeshInterpolator.h"
#include <Eigen/Core>

class Polygon2DAccessor;
class TopologyFace;

// Projects a source mesh on a target mesh. Source and target are both flat
// two dimensional meshes. The source can be either a Polygon2D or a Polygon3D
// which makes this interpolator more flexible. If its of type Polygon3D, the
// outer mesh is used for projection.
// The target mesh must be of type Polygon2D.
//
// Call solveNewton() or solveNCG() before calling update().
//
// The targets Polygon2Ds positions are updated by calling update().
class MeshInterpolatorMeshMesh : public MeshInterpolator
{
public:

    struct NCGParameters
    {
        // \param _maxIterations - the maximum number of solver iterations.
        //      Too low values can result in worse results, too high values
        //      are often problematic in situations where there is no solution
        //      because then, the maxmimum number of iterations will always be
        //      reached. This effect could be reduced by finding a good value
        //      for _minUpdateStepSize.
        // \param _conjugateIterationResetThreshold - the number of iterations
        //      that are used to find search directions w.r.t the conjugate
        //      direction. If this value is 0, the algorithm becomes gradient
        //      descent. This value should not be higher than the number of
        //      parameters.
        // \param _maxProjectionError - the maximum allowed projection error.
        //      The solver terminates early if the projection error with the
        //      current set of parameters is below this value. Lower values
        //      guarantee more accurate results (if maxIterations aren't reached)
        //      but more iterations may be required.
        // \param _minUpdateStepSize - the minimum allowed update step size.
        //      If the update step size is smaller than this value, the solver
        //      terminates. Because the projection error will be higher than
        //      _maxProjectionError, it is considered to not have converged.
        //      This value can be used to early terminate the solver if it is
        //      expected that there is no solution (instead of letting it run
        //      for the full number of maximum iterations.) This value must be
        //      chosen carefully, because early terminating the solving when
        //      there is a solution (that just requires many small steps) can
        //      lead to bad results. The lower the value, the safer. The higher
        //      the faster. Use std::numeric_limist<double>::min() to deactivate
        //      early termination.
        NCGParameters(
                size_t _maxIterations = 2000,
                size_t _conjugateIterationResetThreshold = 4,
                double _maxProjectionError = 1e-6,
                double _minUpdateStepSize = std::numeric_limits<double>::min())
            : maxIterations(_maxIterations)
            , conjugateIterationResetThreshold(_conjugateIterationResetThreshold)
            , maxProjectionError(_maxProjectionError)
            , minUpdateStepSize(_minUpdateStepSize)
        {

        }

        size_t maxIterations;

        // After this amount of iterations, the search direction is reset
        // to the gradient.
        // If this value is 0, this approach becomes equivalent to gradient
        // descent.
        size_t conjugateIterationResetThreshold;

        // Maximum accepted error. The error is the norm of the gradient.
        double maxProjectionError;

        double minUpdateStepSize;
    };

    struct NewtonParameters
    {
        // See @NonlinearConjugateGradientParameters for explanations on what
        // the parameters mean.
        NewtonParameters(
                size_t _maxIterations = 10,
                double _maxProjectionError = 1e-6,
                double _minUpdateStepSize = std::numeric_limits<double>::min())
            : maxIterations(_maxIterations)
            , maxProjectionError(_maxProjectionError)
            , minUpdateStepSize(_minUpdateStepSize)
        {

        }

        size_t maxIterations;

        double maxProjectionError;

        double minUpdateStepSize;
    };

    struct LineSearchParameters
    {
        LineSearchParameters(
                size_t _maxIterations = 10,
                double _initialStepSize = 1.0,
                double _c = 0.1,
                double _tau = 0.5)
            : maxIterations(_maxIterations)
            , initialStepSize(_initialStepSize)
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

        double initialStepSize;

        // acceptance value
        // The higher this value, the harder it is for a value to be accepted.
        // If this value is too high, the line search is more likely to
        // search for a better suited value until the max iteration count is
        // reached. If the value is too low, worse step sizes could be accepted,
        // terminating the line search too early
        double c;

        // Factor with which the step size is reduced in each line search step.
        double tau;
    };

    // Projects source to target.

    // \param source - the source mesh
    MeshInterpolatorMeshMesh(const std::shared_ptr<Polygon>& source,
                             const std::shared_ptr<Polygon>& target);

    virtual ~MeshInterpolatorMeshMesh() override;

    // Use Newtons method to solve the nonlinear equations. This is the
    // prefered solver because it converges a lot faster compared to NCG.
    void solveNewton(const NewtonParameters& newtonParams = NewtonParameters());

    // Use the Nonlinear Conjugate Gradient method to solve the nonlinear
    // equations. This solver converges a lot slower than Newtons method
    // (in our tests ~100 times faster while each iterations is also faster
    // to calculate for this simple 3 dimensional problem).
    // Its only left in here for educational purposes. Calling solveNewton()
    // should be preferred.
    void solveNCG(
            const NCGParameters& ncgParams = NCGParameters(),
            const LineSearchParameters& lsParams = LineSearchParameters());

    // Solves the system using the newton method with the default parameters.
    virtual void solve() override;

    // Calls geometricDataChanged() of target.
    virtual void update() override;

    virtual Type getType() const override;

    // Returns the position of the source vertex that correspondons to the
    // target vertex with the given id.
    virtual Eigen::Vector3d getInterpolatedPosition(size_t targetId) const override;

    // Returns the id of the source vertex w.r.t. the target vertex with the
    // given id.
    virtual size_t getSourceFaceId(size_t targetId) const;

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
                            double distanceBounded,
                            std::size_t sourceFaceId,
                            std::size_t targetVertexId,
                            bool converged)
            : mWeights(weights)
            , mDistance(distance)
            , mDistanceBounded(distanceBounded)
            , mSourceFaceId(sourceFaceId)
            , mTargetVertexId(targetVertexId)
            , mConverged(converged)
            , mAssigned(true)
        {

        }

        // Baryzentric cooridnates w.r.t. the low res mesh.
        Eigen::Vector3d mWeights;

        // Distance of the point on the low res mesh to the one on the high res mesh.
        double mDistance;

        // Distance with bounded baryzentric coordinates.
        double mDistanceBounded;

        // The id of the corresponding vertex on the low res mesh.
        std::size_t mSourceFaceId;

        std::size_t mTargetVertexId;

        bool mConverged;

        bool mAssigned;

    };

    void solve(
            const std::function<Eigen::Vector3d(
                Eigen::Vector3d p,
                Eigen::Vector3d v[3],
                Eigen::Vector3d n[3],
                bool& converged,
                bool printStatistics)>& solverFunction);

    // Nonliner conjugate gradient
    // Use
    Eigen::Vector3d calculateWeightsdNCG(
            Eigen::Vector3d p,
            Eigen::Vector3d v[3],
            Eigen::Vector3d n[3],
            Eigen::Vector4d weights,
            bool& converged,
            const NCGParameters& gdParams,
            const LineSearchParameters& lsParams,
            bool printStatistics = false);

    Eigen::Vector3d calculateWeightsdNewton(
            Eigen::Vector3d p,
            Eigen::Vector3d v[3],
            Eigen::Vector3d n[3],
            Eigen::Vector4d weights,
            bool& converged,
            const NewtonParameters& newtonParams,
            bool printStatistics = false);

    Eigen::Vector3d interpolate(
            const VertexInterpolation& interpolation);

    Eigen::Vector3d interpolate(
            const Eigen::Vector3d& p,
            Eigen::Vector3d v[3],
            Eigen::Vector3d n[3],
            const Eigen::Vector3d& weights);

    Eigen::Vector3d calcA(
            const Eigen::Vector3d& p,
            Eigen::Vector3d v[3],
            Eigen::Vector4d weights)
    {
        return p - (weights[0] * v[0]
                + weights[1] * v[1]
                + weights[2] * v[2]);
    }

    // with weights \in R^3
    Eigen::Vector3d calcA(
            const Eigen::Vector3d& p,
            Eigen::Vector3d v[3],
            Eigen::Vector3d weights)
    {
        return p - (weights[0] * v[0]
                + weights[1] * v[1]
                + weights[2] * v[2]);
    }

    Eigen::Vector3d calcB(
            Eigen::Vector3d n[3],
            Eigen::Vector4d weights)
    {
        return weights[0] * n[0]
                + weights[1] * n[1]
                + weights[2] * n[2];
    }

    // with weights \in R^3
    Eigen::Vector3d calcB(
            Eigen::Vector3d n[3],
            Eigen::Vector3d weights)
    {
        return weights[0] * n[0]
                + weights[1] * n[1]
                + weights[2] * n[2];
    }

    // Calculates F \in \mathbb{R}^3
    Eigen::Vector3d calcF(
            const Eigen::Vector3d& A,
            const Eigen::Vector3d& B)
    {
        return A.cross(B);
    }

    // Calculates \frac{\partial f}{\partial \alpha} \in \mathbb{R}^3
    // \param fGradReturn - return value for the three gradients.
    void calcFGrad(
            Eigen::Vector3d v[3],
            Eigen::Vector3d n[3],
            const Eigen::Vector3d& A,
            const Eigen::Vector3d& B,
            Eigen::Vector3d fGradReturn[3])
    {
        for (size_t i = 0; i < 3; ++i)
        {
            fGradReturn[i] = B.cross(v[i]) + A.cross(n[i]);
        }
    }

    // Calculates \frac{\partial L}{\partial \alpha, \lambda} \in \mathbb{R}^4
    // \Returns Eigen::Vector4d
    Eigen::Vector4d calcLGrad(
            const Eigen::Vector3d& F,
            Eigen::Vector3d fGrad[3],
            const Eigen::Vector4d& weights)
    {
        Eigen::Vector3d partialF;
        for (Eigen::Index i = 0; i < 3; ++i)
        {
            partialF(i) = 2 * F.transpose() * fGrad[i] + weights[3];
        }
        double partialLambda = weights[0] + weights[1] + weights[2] - 1;
        Eigen::Vector4d returnValue;
        returnValue << partialF, partialLambda;
        return returnValue;
    }

    // Calculates \frac{\partial F}{\partial \alpha\alpha} \in \mathbb{R}^{3,3,3}
    // \param returnValue - Eigen::Vector3d[3][3]
    void calcFGradGrad(
            Eigen::Vector3d v[3],
            Eigen::Vector3d n[3],
            Eigen::Vector3d returnValue[3][3])
    {
        // TODO: symmetric, can be optimized
        for (Eigen::Index i = 0; i < 3; ++i)
        {
            for (Eigen::Index j = 0; j < 3; ++j)
            {
                returnValue[i][j] = n[i].cross(v[j]) + n[j].cross(v[i]);
            }
        }
    }

    // Calculates \frac{\partial L}{\partial \alpha\alpha} \in \mathbb{R}^{3,3}.
    // \Returns Eigen::Matrix3d
    Eigen::Matrix3d calcLGradGrad(
            const Eigen::Vector3d& F,
            Eigen::Vector3d FGrad[3],
            Eigen::Vector3d FGradGrad[3][3])
    {
        Eigen::Matrix3d returnValue;
        for (Eigen::Index i = 0; i < 3; ++i)
        {
            for (Eigen::Index j = 0; j < 3; ++j)
            {
                returnValue(i, j) = 2 * (
                            static_cast<double>(FGrad[i].transpose() * FGrad[j]) +
                            static_cast<double>(F.transpose() * FGradGrad[i][j]));
            }
        }
        return returnValue;
    }

    // Calculates the function h \in \mathbb{R}
    double calcH(const Eigen::Vector4d& LGrad)
    {
        return LGrad(0) * LGrad(0)
                + LGrad(1) * LGrad(1)
                + LGrad(2) * LGrad(2)
                + LGrad(3) * LGrad(3);
    }

    // Calculates \frac{\partial h}{\partial \alpha\lambda} \in \mathbb{R}^4
    Eigen::Vector4d calcHGrad(
            const Eigen::Vector4d& LGrad,
            const Eigen::Matrix3d& LGradGrad)
    {
        Eigen::Vector4d returnValue;
        for (Eigen::Index i = 0; i < 3; ++i)
        {
            returnValue(i) = 2 * (LGrad(0) * LGradGrad(0, i)
                                  + LGrad(1) * LGradGrad(1, i)
                                  + LGrad(2) * LGradGrad(2, i)
                                  + LGrad(3));
        }
        returnValue(3) = 2 * (LGrad(0) + LGrad(1) + LGrad(2));
        return returnValue;
    }

    // Returns double
    // Same as calcH but all in one method.
    double calcH2(
            const Eigen::Vector3d& p,
            Eigen::Vector3d v[3],
            Eigen::Vector3d n[3],
            const Eigen::Vector4d& weights)
    {
        Eigen::Vector3d A = calcA(p, v, weights);
        Eigen::Vector3d B = calcB(n, weights);
        Eigen::Vector3d F = calcF(A, B);

        Eigen::Vector3d FGrad[3];
        calcFGrad(v, n, A, B, FGrad);

        Eigen::Vector4d LGrad = calcLGrad(F, FGrad, weights);
        return calcH(LGrad);
    }

    double calcL2(
            const Eigen::Vector3d& p,
            Eigen::Vector3d v[3],
            Eigen::Vector3d n[3],
            const Eigen::Vector4d& weights)
    {
        Eigen::Vector3d f = calcA(p, v, weights).cross(calcB(n, weights));
        double g = weights[0] + weights[1] + weights[2] - 1;
        return static_cast<double>(f.norm() - weights[3] * g);
    }

    double calcG(double h)
    {
        return sqrt(h);
    }

    Eigen::Vector4d calcGGrad(double g, const Eigen::Vector4d& hGrad)
    {
        return (1.0 / (2.0 * g)) * hGrad;
    }

    double calcG2(const Eigen::Vector3d& p,
                  Eigen::Vector3d v[3],
                  Eigen::Vector3d n[3],
                  const Eigen::Vector4d& weights)
    {
        return calcG(calcH2(p, v, n, weights));
    }

    bool mSolved;

    // Baryzentric cooridnates w.r.t. the low res mesh.
    // There is one per point of the high res mesh.
    std::vector<VertexInterpolation> mInterpolations;
};

#endif // MESHINTERPOLATORMESHMESH_H
