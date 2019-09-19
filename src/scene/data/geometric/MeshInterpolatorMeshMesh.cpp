#include "MeshInterpolatorMeshMesh.h"
#include "Polygon.h"
#include "Polygon2D.h"
#include "Polygon2DTopology.h"
#include "PolygonData.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <numeric>


MeshInterpolatorMeshMesh::MeshInterpolatorMeshMesh(
        const std::shared_ptr<Polygon>& source,
        const std::shared_ptr<Polygon>& target)
    : MeshInterpolator (source, target)
    , mSolved(false)
{
}

MeshInterpolatorMeshMesh::~MeshInterpolatorMeshMesh()
{

}

void MeshInterpolatorMeshMesh::solveNewton(
        const MeshInterpolatorMeshMesh::NewtonParameters& newtonParams)
{
    std::function<Eigen::Vector3d(
                Vector3d p,
                Vector3d v[3],
                Vector3d n[3],
                bool& converged,
                bool printStatistics)> f =
            [this, newtonParams](
            Vector3d p,
            Vector3d v[3],
            Vector3d n[3],
            bool& converged,
            bool printStatistics)
    {
        return this->calculateWeightsdNewton(
                    p, v, n,
                    Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                    converged,
                    newtonParams,
                    printStatistics);
    };

    solve(f);

    // Version without lambda. Could be that this is better supported by the
    // microsoft compiler. Is equivalent to the one above but more verbose.
//    std::function<Eigen::Vector3d(
//                MeshInterpolatorMeshMesh*,
//                MeshInterpolatorMeshMesh::NewtonParameters,
//                Vector3d p,
//                Vector3d v[3],
//                Vector3d n[3],
//                bool& converged,
//                bool printStatistics)> fTemp =
//            [](
//            MeshInterpolatorMeshMesh* interpolator,
//            MeshInterpolatorMeshMesh::NewtonParameters params,
//            Vector3d p,
//            Vector3d v[3],
//            Vector3d n[3],
//            bool& converged,
//            bool printStatistics)
//    {
//        return interpolator->calculateWeightsdNewton(
//                    p, v, n,
//                    Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
//                    converged,
//                    params,
//                    printStatistics);
//    };

//    std::function<Eigen::Vector3d(
//                Vector3d p,
//                Vector3d v[3],
//                Vector3d n[3],
//                bool& converged,
//                bool printStatistics)> f =
//            std::bind(
//                fTemp, this, newtonParams,
//                std::placeholders::_1,
//                std::placeholders::_2,
//                std::placeholders::_3,
//                std::placeholders::_4,
//                std::placeholders::_5);
}

void MeshInterpolatorMeshMesh::solveNCG(
        const MeshInterpolatorMeshMesh::NCGParameters& ncgParams,
        const MeshInterpolatorMeshMesh::LineSearchParameters& lsParams)
{
    std::function<Eigen::Vector3d(
                Vector3d p,
                Vector3d v[3],
                Vector3d n[3],
                bool& converged,
                bool printStatistics)> f =
            [this, ncgParams, lsParams](
            Vector3d p,
            Vector3d v[3],
            Vector3d n[3],
            bool& converged,
            bool printStatistics)
    {
        return this->calculateWeightsdNCG(
                    p, v, n,
                    Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                    converged,
                    ncgParams,
                    lsParams,
                    printStatistics);
    };

    solve(f);

    // Version without lambda. Could be that this is better supported by the
    // microsoft compiler. Is equivalent to the one above but more verbose.
//    std::function<Eigen::Vector3d(
//                MeshInterpolatorMeshMesh*,
//                MeshInterpolatorMeshMesh::NCGParameters,
//                MeshInterpolatorMeshMesh::LineSearchParameters,
//                Vector3d p,
//                Vector3d v[3],
//                Vector3d n[3],
//                bool& converged,
//                bool printStatistics)> fTemp =
//            [](
//            MeshInterpolatorMeshMesh* interpolator,
//            MeshInterpolatorMeshMesh::NCGParameters ncgParams,
//            MeshInterpolatorMeshMesh::LineSearchParameters lsParams,
//            Vector3d p,
//            Vector3d v[3],
//            Vector3d n[3],
//            bool& converged,
//            bool printStatistics)
//    {
//        return interpolator->calculateWeightsdNCG(
//                    p, v, n,
//                    Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
//                    converged,
//                    ncgParams,
//                    lsParams,
//                    printStatistics);
//    };

//    std::function<Eigen::Vector3d(
//                Vector3d p,
//                Vector3d v[3],
//                Vector3d n[3],
//                bool& converged,
//                bool printStatistics)> f =
//            std::bind(
//                fTemp, this, ncgParams, lsParams,
//                std::placeholders::_1,
//                std::placeholders::_2,
//                std::placeholders::_3,
//                std::placeholders::_4,
    //                std::placeholders::_5);
}

void MeshInterpolatorMeshMesh::solve()
{
    solveNewton();
}

void MeshInterpolatorMeshMesh::update()
{
    if (!mSolved)
    {
        std::cout << "Tried calling MeshInterpolatorMeshMesh::update() before "
                     "calling MeshInterpolatorMeshMesh::solveNewton().\n";
        return;
    }
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
    mTarget->update();
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

size_t MeshInterpolatorMeshMesh::getSourceFaceId(size_t targetId) const
{
    return mInterpolations[targetId].mSourceFaceId;
}

void MeshInterpolatorMeshMesh::solve(
        const std::function<Vector3d
        (Vector3d, Vector3d[], Vector3d[], bool&, bool)>& solverFunction)
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

    mInterpolations.clear();
    mInterpolations.reserve(mTarget->getSize());

    double wnt = 1e-2; // weight normal tolerance
    double wbt = 1e-1; // weight bounding tolerance
    double wt = 1e-2; // weight tolerance
    bool printStatistics = false;
    bool printErrors = false;

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

            bool isInside =
                    ((p - v[0]).dot(fn[0]) > -wnt || (p - v[1]).dot(fn[1]) > -wnt) &&
                    ((p - v[1]).dot(fn[2]) > -wnt || (p - v[2]).dot(fn[3]) > -wnt) &&
                    ((p - v[2]).dot(fn[4]) > -wnt || (p - v[0]).dot(fn[5]) > -wnt);

            if (!isInside)
            {
                continue;
            }

            bool converged;

            Eigen::Vector3d weights = solverFunction(
                        p, v, n,
                        converged,
                        printStatistics);

//            Eigen::Vector3d weights = calculateWeightsdNewton(
//                        p, v, n,
//                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
//                        converged,
//                        NewtonParameters(10, 1e-6),
//                        printStatistics);

//            Eigen::Vector3d weights = calculateWeightsdNewton(
//                        p, v, n, Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
//                        converged,
//                        NewtonParameters(10, 1e-5, 1e-6),
//                        LineSearchParameters(20, 1.0 , 0.1, 0.25), printStatistics);

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

            bool inBounds =
//                    std::abs(weights(0) + weights(1) + weights(2) - 1) < 3 * wt &&
                    -wbt <= weights[0] && weights[0] <= 1.0 + wbt &&
                    -wbt <= weights[1] && weights[1] <= 1.0 + wbt &&
                    -wbt <= weights[2] && weights[2] <= 1.0 + wbt;

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
            if (printErrors)
                std::cout << "warning: interpolation for vertex at " << i << "("
                          << p.transpose() << ") did not converge.\n";
        }
        else if (!outOfBoundsInterpolations.empty())
        {
            chosenInterpolations = &outOfBoundsInterpolations;
            if (printErrors)
                std::cout << "warning: interpolation for vertex at " << i << "("
                          << p.transpose() << ") is out of bounds.\n";
        }
        else
        {
            chosenInterpolations = &allInterpolations;
            if (printErrors)
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
            VertexInterpolation chosen = (*chosenInterpolations)[0];

            mInterpolations.push_back(chosen);


            Eigen::Vector target = interpolate(chosen);
            double error = (target - p).norm();

            if (error > wt)
            {
                errors.push_back(error);
                if (printErrors)
                    std::cout << "error = " << error
                              << ", p_orig = " << p.transpose()
                              << ", p_proj = " << target.transpose() << "\n";
            }
        }
        else
        {
            std::cout << "warning: No candidate vertex found for vertex at " << i << "\n";
        }

    }
    std::cout << "errors size = " << errors.size() << "\n";
    std::sort(errors.begin(), errors.end());
    for (size_t i = 0; i < errors.size(); ++i)
    {
        std::cout << errors[i] << "\n";
    }
    mSolved = true;
}

Vector3d MeshInterpolatorMeshMesh::calculateWeightsdNCG(
        Vector3d p,
        Vector3d v[],
        Vector3d n[],
        Vector4d weights,
        bool& converged,
        const MeshInterpolatorMeshMesh::NCGParameters& ncgParams,
        const MeshInterpolatorMeshMesh::LineSearchParameters& lsParams,
        bool printStatistics)
{
    converged = false;

    Eigen::Vector4d s; // search direction

    Eigen::Vector4d deltaX;
    Eigen::Vector4d deltaXPrev;

    size_t iterationsSinceLastReset = 0;

    // Gradient Descent
    double error = 1.0;
    size_t iterTotal = 0;
    for (size_t iterCount = 0; iterCount < ncgParams.maxIterations; ++iterCount)
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

        deltaX = -HGrad;

        if (iterCount == 0 ||
            iterationsSinceLastReset == ncgParams.conjugateIterationResetThreshold)
        {
            s = deltaX;
            iterationsSinceLastReset = 0;
        }
        else
        {
            double beta = deltaX.dot(deltaX - deltaXPrev) / deltaXPrev.norm();
            if (beta < 1e-8)
            {
                s = deltaX;
                iterationsSinceLastReset = 0;
            }
            else
            {
                s = deltaX + beta * s;
                iterationsSinceLastReset++;
            }
        }

        Eigen::Vector4d dir = s.normalized(); // search direction
        // Perform line search to find optimal t.
        double m = dir.dot(deltaX);
        // 1 / lsParams.tau because to counteract the multiplication in the
        // first iteration.
        double t = 1 / lsParams.tau * lsParams.initialStepSize;
        double HNextMin = std::numeric_limits<double>::max();
        double tMin = t;
        for (size_t j = 0; j < lsParams.maxIterations; ++j)
        {
            t *= lsParams.tau;
            double Hnext = calcH2(p, v, n, weights + t * dir);

            if (HNextMin > Hnext)
            {
                HNextMin = Hnext;
                tMin = t;
            }

            if (std::abs(H) - std::abs(Hnext) >= lsParams.c * m)
                break;
        }

        // Gradient Descent update
        Eigen::Vector4d delta = tMin * dir;
        weights += delta;

        error = delta.norm();

        // improved convergence criterium
        Eigen::Vector pProj = interpolate(p, v, n,
                                          Eigen::Vector3d(weights(0),
                                                          weights(1),
                                                          weights(2)));
        double projError = (p - pProj).norm();

        // convergence criterium
        if (projError < ncgParams.maxProjectionError)
        {
            converged = true;
            break;
        }

        // non-convergence criterium
        if (error < ncgParams.minUpdateStepSize)
        {
            converged = true;
            break;
        }

        deltaXPrev = deltaX;
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

Vector3d MeshInterpolatorMeshMesh::calculateWeightsdNewton(
        Vector3d p,
        Vector3d v[3],
        Vector3d n[3],
        Vector4d _weights,
        bool& converged,
        const MeshInterpolatorMeshMesh::NewtonParameters& newtonParams,
        bool printStatistics)
{
    converged = false;

    // Gradient Descent
    double error = 1.0;
    size_t iterTotal = 0;

    Vector3d weights = Vector3d(_weights(0), _weights(1), _weights(2));
    for (size_t iterCount = 0; iterCount < newtonParams.maxIterations; ++iterCount)
    {
        ++iterTotal;
        Eigen::Vector A = calcA(p, v, weights);
        Eigen::Vector B = calcB(n, weights);
        Eigen::Vector F = calcF(A, B);

        // calc f
        double g = weights[0] + weights[1] + weights[2] - 1;
        Eigen::Vector4d f;
        f << F, g;

        // calc fGrad
        Eigen::Vector3d FGrad[3];
        calcFGrad(v, n, A, B, FGrad);
        Eigen::Matrix<double, 4, 3> fGrad;
        fGrad << FGrad[0](0), FGrad[1](0), FGrad[2](0),
                FGrad[0](1), FGrad[1](1), FGrad[2](1),
                FGrad[0](2), FGrad[1](2), FGrad[2](2),
                1, 1, 1;

        // calc pseudo inverse
        Eigen::Matrix3d m = (fGrad.transpose() * fGrad);
        Eigen::Matrix3d mInv = m.inverse();

        // update
        Eigen::Vector delta = -mInv * fGrad.transpose() * f;

        weights += delta;

        // improved convergence criterium
        Eigen::Vector pProj = interpolate(p, v, n,
                                          Eigen::Vector3d(weights(0),
                                                          weights(1),
                                                          weights(2)));
        double projError = (p - pProj).norm();

        if (projError < newtonParams.maxProjectionError)
        {
            converged = true;
            break;
        }

        if (error < newtonParams.minUpdateStepSize)
        {
            converged = false;
            break;
        }
    }

    if (printStatistics)
    {
        std::cout << "Iterations: " << iterTotal
                  << ", error: " << error
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

Vector3d MeshInterpolatorMeshMesh::interpolate(
        const Eigen::Vector3d& p,
        Vector3d v[],
        Vector3d n[],
        const Vector3d& weights)
{
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

    return q + distance * n_q;
}
