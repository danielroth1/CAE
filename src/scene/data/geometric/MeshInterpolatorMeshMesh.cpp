#include "MeshInterpolatorMeshMesh.h"
#include "Polygon.h"
#include "Polygon2D.h"
#include "Polygon2DTopology.h"
#include "PolygonData.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>


MeshInterpolatorMeshMesh::GradientDescentParameters
MeshInterpolatorMeshMesh::GradientDescentParameters::DEFAULT_GD_PARAMS;

MeshInterpolatorMeshMesh::LineSearchParameters
MeshInterpolatorMeshMesh::LineSearchParameters::DEFAULT_LS_PARAMS;

MeshInterpolatorMeshMesh::MeshInterpolatorMeshMesh(
        const std::shared_ptr<Polygon>& source,
        const std::shared_ptr<Polygon>& target)
    : MeshInterpolator (source, target)
{
    init();
}

void MeshInterpolatorMeshMesh::update()
{
    for (size_t i = 0; i < mTargetAccessor->getSize(); ++i)
    {
        VertexInterpolation& vi = mInterpolations[i];
        if (vi.mAssigned)
        {
            Face& f = mSourceAccessor->getTopology2D().getFace(vi.mSourceFaceId).getVertexIds();
            Eigen::Vector q =
                    vi.mWeights[0] * mSourceAccessor->getPosition(f[0])
                    + vi.mWeights[1] * mSourceAccessor->getPosition(f[1])
                    + vi.mWeights[2] * mSourceAccessor->getPosition(f[2]);
            Eigen::Vector n_q =
                    vi.mWeights[0] * mSourceAccessor->getVertexNormals()[f[0]]
                    + vi.mWeights[1] * mSourceAccessor->getVertexNormals()[f[1]]
                    + vi.mWeights[2] * mSourceAccessor->getVertexNormals()[f[2]];

            n_q.normalize();

            Eigen::Vector p = q + vi.mDistance * n_q;
            mTargetAccessor->setPosition(i, p);
        }
    }
    mTarget->geometricDataChanged();
}

size_t MeshInterpolatorMeshMesh::getSourceFaceId(size_t targetId) const
{
    return mInterpolations[targetId].mSourceFaceId;
}

Vector3d MeshInterpolatorMeshMesh::getSourcePosition(size_t targetId) const
{
    const VertexInterpolation& vi = mInterpolations[targetId];
    Face& f = mSourceAccessor->getTopology2D().getFace(vi.mSourceFaceId).getVertexIds();
    Eigen::Vector q =
            vi.mWeights[0] * mSourceAccessor->getPosition(f[0])
            + vi.mWeights[1] * mSourceAccessor->getPosition(f[1])
            + vi.mWeights[2] * mSourceAccessor->getPosition(f[2]);
    return q;
}

void MeshInterpolatorMeshMesh::init()
{
    mSource->update();
    mTarget->update();

    if (mSource->getPositionType() == BSWSVectors::Type::BODY_SPACE &&
        mTarget->getPositionType() == BSWSVectors::Type::WORLD_SPACE)
    {
        mTarget->changeRepresentationToBS(mTarget->calculateCenterVertex());
    }
    else if (mSource->getPositionType() == BSWSVectors::Type::WORLD_SPACE &&
             mTarget->getPositionType() == BSWSVectors::Type::BODY_SPACE)
    {
        mTarget->changeRepresentationToWS();
    }

    mInterpolations.reserve(mTarget->getSize());

    double wbt = 1e-3; // weight bounding tolerance
    double wt = 1e-2; // weight tolerance
    bool printStatistics = true;

    std::vector<double> errors;

    size_t entriesPerPercent = std::max(static_cast<size_t>(1), static_cast<size_t>(std::floor(mTargetAccessor->getSize() / 100.0)));
    for (size_t i = 0; i < mTargetAccessor->getSize(); ++i)
    {
        if (i % entriesPerPercent == 0)
        {
            std::cout << std::round(static_cast<double>(i) / mTargetAccessor->getSize() * 100.0)
                      << " / 100\n";
        }
        std::vector<VertexInterpolation> candidateInterpolations;
        std::vector<VertexInterpolation> outOfBoundsInterpolations;
        std::vector<VertexInterpolation> nonConvergedInterpolations;
        std::vector<VertexInterpolation> allInterpolations;

        Eigen::Vector p = mTargetAccessor->getPosition(i);

        // find q, n_q, and distance so that distance is minimal

        Polygon2DTopology& topology = mSourceAccessor->getTopology2D();
        for (size_t fId = 0; fId < topology.getFaces().size(); ++fId)
        {
            TopologyFace& f = topology.getFace(fId);

            // vertices per face

            Eigen::Vector n[3];
            n[0] = mSourceAccessor->getVertexNormals()[f.getVertexIds()[0]];
            n[1] = mSourceAccessor->getVertexNormals()[f.getVertexIds()[1]];
            n[2] = mSourceAccessor->getVertexNormals()[f.getVertexIds()[2]];

            Eigen::Vector v[3];
            v[0] = mSourceAccessor->getPosition(f.getVertexIds()[0]);
            v[1] = mSourceAccessor->getPosition(f.getVertexIds()[1]);
            v[2] = mSourceAccessor->getPosition(f.getVertexIds()[2]);

            // check if vertex is in range
            Eigen::Vector fn[6];
            fn[0] = (v[1] - v[0]).cross(n[0]);
            fn[1] = (v[0] - v[1]).cross(n[1]);
            fn[2] = (v[2] - v[1]).cross(n[1]);
            fn[3] = (v[1] - v[2]).cross(n[2]);
            fn[4] = (v[0] - v[2]).cross(n[2]);
            fn[5] = (v[2] - v[0]).cross(n[0]);

            for (size_t j = 0; j < 6; ++j)
            {
                fn[j].normalize();
            }

            if (fn[0].dot(v[2] - v[0]) < 0)
                fn[0] *= -1;
            if (fn[1].dot(v[2] - v[1]) < 0)
                fn[1] *= -1;
            if (fn[2].dot(v[0] - v[1]) < 0)
                fn[2] *= -1;
            if (fn[3].dot(v[0] - v[2]) < 0)
                fn[3] *= -1;
            if (fn[4].dot(v[1] - v[2]) < 0)
                fn[4] *= -1;
            if (fn[5].dot(v[1] - v[0]) < 0)
                fn[5] *= -1;

//            for (size_t j = 0; j < 2; ++j)
//            {
//                if (fn[j].dot(v[2]) < 0)
//                    fn[j] *= -1;
//                if (fn[j+2].dot(v[0]) < 0)
//                    fn[j+2] *= -1;
//                if (fn[j+4].dot(v[1]) < 0)
//                    fn[j+4] *= -1;
//            }

            bool isInside =
                    ((p - v[0]).dot(fn[0]) > 0 || (p - v[1]).dot(fn[1]) > 0) &&
                    ((p - v[1]).dot(fn[2]) > 0 || (p - v[2]).dot(fn[3]) > 0) &&
                    ((p - v[2]).dot(fn[4]) > 0 || (p - v[0]).dot(fn[5]) > 0);

            if (!isInside)
            {
//                std::cout << "is not inside so continue\n";
                continue;
            }

            bool converged;
            Eigen::Vector3d weights = calculateWeightsImprovedGD(
                        p, v, n, Eigen::Vector4d(0.4, 0.3, 0.3, 0.0),
                        converged,
                        GradientDescentParameters(10000, 1e-6),
                        LineSearchParameters(20, 0.1, 0.25), printStatistics);

            // signed distance
            Eigen::Vector q = weights[0] * v[0]
                    + weights[1] * v[1]
                    + weights[2] * v[2];
            Eigen::Vector n_q = weights[0] * n[0]
                    + weights[1] * n[1]
                    + weights[2] * n[2];
            n_q.normalize();
            Eigen::Vector r = p - q;

            double sign = r.dot(n_q) < 0 ? -1 : 1;
            double distance = sign * r.norm();

            VertexInterpolation interpolation(Eigen::Vector3d(
                                                  weights[0],weights[1], weights[2]),
                    distance, fId, i);
            allInterpolations.push_back(interpolation);

            double error = (q + distance * n_q - p).norm();
            bool inBounds =
//                    error < wt &&
//                    std::abs(weights(0) + weights(1) + weights(2) - 1) < 3 * wt &&
                    -wbt <= weights[0] && weights[0] <= 1.0 + wbt &&
                    -wbt <= weights[1] && weights[1] <= 1.0 + wbt &&
                    -wbt <= weights[2] && weights[2] <= 1.0 + wbt;
//            weights.normalize();

            if (converged && inBounds)
            {
                candidateInterpolations.push_back(interpolation);
            }
            else if (!converged && inBounds)
            {
                nonConvergedInterpolations.push_back(interpolation);
            }
            else if (converged && !inBounds)
            {
                outOfBoundsInterpolations.push_back(interpolation);
            }

            allInterpolations.push_back(interpolation);
        }

        std::vector<VertexInterpolation>* chosenInterpolations;

        if (!candidateInterpolations.empty())
        {
            chosenInterpolations = &candidateInterpolations;
        }
        else if (!nonConvergedInterpolations.empty())
        {
            chosenInterpolations = &nonConvergedInterpolations;
            std::cout << "warning: interpolation for vertex at " << i << "("
                      << p.transpose() << ") did not converge.\n";
        }
        else if (!outOfBoundsInterpolations.empty())
        {
            chosenInterpolations = &outOfBoundsInterpolations;
            std::cout << "warning: interpolation for vertex at " << i << "("
                      << p.transpose() << ") is out of bounds.\n";
        }
        else
        {
            chosenInterpolations = &allInterpolations;
            std::cout << "warning: interpolation for vertex at " << i << "("
                      << p.transpose() << ") did not converge and is out of bounds.\n";
        }

        std::sort(chosenInterpolations->begin(),
                  chosenInterpolations->end(),
                  [](const VertexInterpolation& vi1,
                  const VertexInterpolation& vi2)
        {
            return std::abs(vi1.mDistance) < std::abs(vi2.mDistance);
        });

        if (!chosenInterpolations->empty())
        {
            mInterpolations.push_back((*chosenInterpolations)[0]);

            VertexInterpolation chosen = (*chosenInterpolations)[0];

            Eigen::Vector target = interpolate(chosen);
            double error = (target - p).norm();

            if (error > wt)
            {
                errors.push_back(error);
                std::cout << "error = " << error
                          << ", p_orig = " << p.transpose()
                          << ", p_proj = " << target.transpose() << "\n";
            }
        }
        else
        {
            std::cout << "warning: No candidate vertex found for vertex at " << i << "\n";
        }

        if (printStatistics)
            std::cout << "==============================\n";

    }
    std::cout << "errors size = " << errors.size() << "\n";
    std::sort(errors.begin(), errors.end());
    for (size_t i = 0; i < errors.size(); ++i)
    {
        std::cout << errors[i] << "\n";
    }
}

Eigen::Vector4d MeshInterpolatorMeshMesh::calculateWeightsSimpleGD(
        Eigen::Vector3d p,
        Eigen::Vector3d v[3],
        Eigen::Vector3d n[3],
        Eigen::Vector4d weights)
{
    auto calcA = [](
            const Eigen::Vector3d& p,
            Eigen::Vector3d v[3],
            Eigen::Vector4d weights)
    {
        return p - (weights[0] * v[0]
                + weights[1] * v[1]
                + weights[2] * v[2]);
    };

    auto calcB = [](
            Eigen::Vector3d n[3],
            Eigen::Vector4d weights)
    {
        return weights[0] * n[0]
                + weights[1] * n[1]
                + weights[2] * n[2];
    };

    auto calcF = [](
            const Eigen::Vector& A,
            const Eigen::Vector& B)
    {
        return A.cross(B);
    };

    auto calcF2 = [calcA, calcB](
                Eigen::Vector3d p,
                Eigen::Vector3d v[3],
                Eigen::Vector3d n[3],
                Eigen::Vector4d weights)
    {
        return calcA(p, v, weights).cross(calcB(n, weights));
    };

    auto calcG = [](
            const Eigen::Vector4d& weights)
    {
        return weights[0] + weights[1] + weights[2] - 1;
    };

    auto calcL = [](
            const Eigen::Vector3d& F,
            const Eigen::Vector4d& weights,
            double g)
    {
//        return static_cast<double>(F.norm() + weights[3] * weights[3] * g * g);
        return static_cast<double>(F.norm() - weights[3] * g);
    };

    auto calcL2 = [calcF2, calcG](
            Eigen::Vector3d p,
            Eigen::Vector3d v[3],
            Eigen::Vector3d n[3],
            Eigen::Vector4d weights)
    {
//        double g = calcG(weights);
//        return static_cast<double>(calcF2(p, v, n, weights).norm() +
//                                   weights[3] * weights[3] * g * g);
        return static_cast<double>(calcF2(p, v, n, weights).norm() -
                                   weights[3] * calcG(weights));
    };

    // Gradient Descent
    double error = 1.0;
    double maxError = 1e-5;
    for (size_t iterCount = 0; iterCount < 10; ++iterCount)
    {
        // convergence criterium
        if (error < maxError)
            break;

        Eigen::Vector A = calcA(p, v, weights);
        Eigen::Vector B = calcB(n, weights);

        // calculate LGrad
        Eigen::Vector F = calcF(A, B);
        double g = calcG(weights);
        double L = calcL(F, weights, g);

        Eigen::Vector FGrad[3];
        for (size_t i = 0; i < 3; ++i)
        {
            FGrad[i] = B.cross(v[i]) + A.cross(n[i]);
        }

        Eigen::Vector4d LGrad;
        for (size_t i = 0; i < 3; ++i)
        {
            LGrad[i] = 2 * F.transpose() * FGrad[i] - weights[3];
//            LGrad[i] = 2 * F.transpose() * FGrad[i] + 2 * weights[3] * weights[3] * g;
        }
        LGrad[3] = -g;

//        if (weights[3] * g < 0)
//            LGrad[3] *= -1;
//        LGrad[3] = 2 * weights[3] * g * g;

        // Perform line search to find optimal t.
        size_t maxLineSearchIterations = 10; // line search parameter
        double c = 0.1; // line search parameter
        double tau = 0.25; // line search parameter
        double m = LGrad.norm();
        double t = 1.0;
        double LNextMin = std::numeric_limits<double>::max();
        double tMin = t;
        for (size_t j = 0; j < maxLineSearchIterations; ++j)
        {
            t *= tau;
            double Lnext = calcL2(p, v, n, weights - t * LGrad);

            if (LNextMin > Lnext)
            {
                LNextMin = Lnext;
                tMin = t;
            }

            if (std::abs(L) - std::abs(Lnext) >= c * m)
                break;
        }

        // Gradient Descent update
        Eigen::Vector4d delta = tMin * LGrad;
        weights -= delta;

        error = delta.norm();
    }
    return weights;
}

Vector3d MeshInterpolatorMeshMesh::calculateWeightsImprovedGD(
        Vector3d p,
        Vector3d v[3],
        Vector3d n[3],
        Vector4d weights,
        bool& converged,
        const GradientDescentParameters& gdParams,
        const LineSearchParameters& lsParams,
        bool printStatistics)
{
    auto calcA = [](
            const Eigen::Vector3d& p,
            Eigen::Vector3d v[3],
            Eigen::Vector4d weights)
    {
        return p - (weights[0] * v[0]
                + weights[1] * v[1]
                + weights[2] * v[2]);
    };

    auto calcB = [](
            Eigen::Vector3d n[3],
            Eigen::Vector4d weights)
    {
        return weights[0] * n[0]
                + weights[1] * n[1]
                + weights[2] * n[2];
    };

    // Calculates F \in \mathbb{R}^3
    auto calcF = [](
            const Eigen::Vector& A,
            const Eigen::Vector& B)
    {
        return A.cross(B);
    };

    // Calculates \frac{\partial f}{\partial \alpha} \in \mathbb{R}^3
    // \param fGradReturn - return value for the three gradients.
    auto calcFGrad = [](
            Eigen::Vector3d v[3],
            Eigen::Vector3d n[3],
            const Eigen::Vector& A,
            const Eigen::Vector& B,
            Eigen::Vector3d fGradReturn[3])
    {
        for (size_t i = 0; i < 3; ++i)
        {
            fGradReturn[i] = B.cross(v[i]) + A.cross(n[i]);
        }
    };

    // Calculates \frac{\partial L}{\partial \alpha, \lambda} \in \mathbb{R}^4
    // \Returns Eigen::Vector4d
    auto calcLGrad = [](
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
    };

    // Calculates \frac{\partial F}{\partial \alpha\alpha} \in \mathbb{R}^{3,3,3}
    // \param returnValue - Eigen::Vector3d[3][3]
    auto calcFGradGrad = [](
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
    };

    // Calculates \frac{\partial L}{\partial \alpha\alpha} \in \mathbb{R}^{3,3}.
    // \Returns Eigen::Matrix3d
    auto calcLGradGrad = [](
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
    };

    // Calculates the function h \in \mathbb{R}
    auto calcH = [](const Eigen::Vector4d& LGrad)
    {
        return LGrad(0) * LGrad(0)
                + LGrad(1) * LGrad(1)
                + LGrad(2) * LGrad(2)
                + LGrad(3) * LGrad(3);
    };

    // Calculates \frac{\partial h}{\partial \alpha\lambda} \in \mathbb{R}^4
    auto calcHGrad = [](
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
    };

    // Returns double
    // Same as calcH but all in one method.
    auto calcH2 = [&calcA, &calcB, &calcF, &calcFGrad, &calcLGrad, &calcH](
            const Vector3d& p,
            Vector3d v[3],
            Vector3d n[3],
            const Vector4d& weights)
    {
        Eigen::Vector A = calcA(p, v, weights);
        Eigen::Vector B = calcB(n, weights);
        Eigen::Vector F = calcF(A, B);

        Eigen::Vector3d FGrad[3];
        calcFGrad(v, n, A, B, FGrad);

        Eigen::Vector4d LGrad = calcLGrad(F, FGrad, weights);
        return calcH(LGrad);
    };

    converged = false;

    // Gradient Descent
    double error = 1.0;
    size_t iterTotal = 0;
    for (size_t iterCount = 0; iterCount < gdParams.maxIterations; ++iterCount)
    {
        ++iterTotal;
        Eigen::Vector A = calcA(p, v, weights);
        Eigen::Vector B = calcB(n, weights);
        Eigen::Vector F = calcF(A, B);

        Eigen::Vector3d FGrad[3];
        calcFGrad(v, n, A, B, FGrad);

        Eigen::Vector4d LGrad = calcLGrad(F, FGrad, weights);

        Eigen::Vector3d FGradGrad[3][3];
        calcFGradGrad(v, n, FGradGrad);

        Eigen::Matrix3d LGradGrad = calcLGradGrad(F, FGrad, FGradGrad);

        double H = calcH(LGrad);

        Eigen::Vector4d HGrad = calcHGrad(LGrad, LGradGrad);

        // Perform line search to find optimal t.
        double m = HGrad.norm();
        double t = 2.0;
        double HNextMin = std::numeric_limits<double>::max();
        double tMin = t;
        for (size_t j = 0; j < lsParams.maxIterations; ++j)
        {
            t *= lsParams.tau;
            double Hnext = calcH2(p, v, n, weights - t * HGrad);

            if (HNextMin > Hnext)
            {
                HNextMin = Hnext;
                tMin = t;
            }

            if (std::abs(H) - std::abs(Hnext) >= lsParams.c * m)
                break;
        }

        // Gradient Descent update
        Eigen::Vector4d delta = tMin * HGrad;
        weights -= delta;

        error = delta.norm();

        // convergence criterium
        if (error < gdParams.accuracy)
        {
            converged = true;
            break;
        }
    }

    if (printStatistics)
    {
        std::cout << "Iterations: " << iterTotal
                  << ", error: " << error
                  << ", lambda: " << weights[3]
                  << ", weights: " << weights(0) << ", " << weights(1) << ", " << weights(2)
                  << "\n";
    }
    return Eigen::Vector3d(weights(0), weights(1), weights(2));
}

Vector3d MeshInterpolatorMeshMesh::interpolate(
        const MeshInterpolatorMeshMesh::VertexInterpolation& interpolation)
{
    TopologyFace& f = mSourceAccessor->getTopology2D().getFace(interpolation.mSourceFaceId);
    Eigen::Vector n[3];
    n[0] = mSourceAccessor->getVertexNormals()[f.getVertexIds()[0]];
    n[1] = mSourceAccessor->getVertexNormals()[f.getVertexIds()[1]];
    n[2] = mSourceAccessor->getVertexNormals()[f.getVertexIds()[2]];
    Eigen::Vector v[3];
    v[0] = mSourceAccessor->getPosition(f.getVertexIds()[0]);
    v[1] = mSourceAccessor->getPosition(f.getVertexIds()[1]);
    v[2] = mSourceAccessor->getPosition(f.getVertexIds()[2]);
    Eigen::Vector q = interpolation.mWeights[0] * v[0]
            + interpolation.mWeights[1] * v[1]
            + interpolation.mWeights[2] * v[2];
    Eigen::Vector n_q = interpolation.mWeights[0] * n[0]
            + interpolation.mWeights[1] * n[1]
            + interpolation.mWeights[2] * n[2];
    n_q.normalize();

    return q + interpolation.mDistance * n_q;
}
