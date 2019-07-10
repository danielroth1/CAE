#include "MeshInterpolatorFEM.h"
#include "Polygon.h"
#include "Polygon3D.h"
#include "Polygon3DTopology.h"

#include <algorithm>
#include <iostream>

MeshInterpolatorFEM::MeshInterpolatorFEM(
        const std::shared_ptr<Polygon3D>& source,
        const std::shared_ptr<Polygon>& target)
    : MeshInterpolator (source, target)
    , mSource3(source)
    , mSolved(false)
{

}

void MeshInterpolatorFEM::solve()
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

    bool printErrors = true;
    double wt = 0.001; // weight tolerance
    std::array<Vector, 4> v; // vertices
    Eigen::Vector4d bary; // baryzentric coordinatees
//    std::array<double, 4> bary; // baryzentric coordinatees
    // Fill mRelevantSourceVertices.
    for (std::size_t i = 0; i < mTarget->getSize(); ++i)
    {
        Eigen::Vector p = mTarget->getPosition(i);

        // filter out elements that do not contain vertex

        // if there are vertices wihtout elemnt, we need to find the element
        // with the closest distance to the vertex.

        std::vector<VertexInterpolation> candidateInterpolations;

        for (size_t j = 0; j < mSource3->getTopology3D().getCells().size(); ++j)
        {
            Cell& c = mSource3->getTopology3D().getCells()[j];

            // Vertex positions
            v[0] = mSource3->getPosition(c[0]);
            v[1] = mSource3->getPosition(c[1]);
            v[2] = mSource3->getPosition(c[2]);
            v[3] = mSource3->getPosition(c[3]);

            Vector r1 = v[1] - v[0];
            Vector r2 = v[2] - v[0];
            Vector r3 = v[3] - v[0];

            Vector r4 = v[2] - v[1];
            Vector r5 = v[1] - v[3];

            double J = r1.cross(r2).dot(r3);

//            double volume = J / 6.0;

            Vector center = 0.25 * (v[0] + v[1] + v[2] + v[3]);

            bary[0] = 0.25 + (p - center).dot(r4.cross(r5) / J);
            bary[1] = 0.25 + (p - center).dot(r2.cross(r3) / J);
            bary[2] = 0.25 + (p - center).dot(r3.cross(r1) / J);
            bary[3] = 0.25 + (p - center).dot(r1.cross(r2) / J);

            VertexInterpolation vi(bary, j, i);
            if (-wt < bary[0] && bary[0] < 1 + wt &&
                -wt < bary[1] && bary[1] < 1 + wt &&
                -wt < bary[2] && bary[2] < 1 + wt &&
                -wt < bary[3] && bary[3] < 1 + wt)
            {
                candidateInterpolations.push_back(vi);
            }
        }

        if (!candidateInterpolations.empty())
        {
            if (printErrors && candidateInterpolations.size() > 1)
            {
                std::cout << "warning: multiple candidate interpolations found for "
                             "vertex with index " << i << ".\n";
            }

            VertexInterpolation vi = candidateInterpolations[0];
            mInterpolations.push_back(vi);

            std::cout << "Adds interpolation with "
                      << "weights = "
                      << vi.mWeights(0) << ", " << vi.mWeights(1) << ", "
                      << vi.mWeights(2) << ", " << vi.mWeights(3) << "\n";
        }
        else if (printErrors)
        {
            std::cerr << "no interpolation found for vertex with index " << i << ".\n";
        }
    }

    mSolved = true;
}

void MeshInterpolatorFEM::update()
{
    if (!mSolved)
    {
        std::cout << "Tried calling MeshInterpolatorMeshMesh::update() before "
                     "calling MeshInterpolatorMeshMesh::solveNewton().\n";
        return;
    }

    // position type must be the same
    if (mSource->getPositionType() !=
        mTarget->getPositionType())
    {
        std::cout << "Not implemented.\n";
        return;
    }

    switch(mSource->getPositionType())
    {
    case BSWSVectors::BODY_SPACE:
    {
        // This is a simple case. Transformation matrices need to be equal.
        mTarget->setTransform(mSource->getTransform());
        break;
    }
    case BSWSVectors::WORLD_SPACE:
    {
        // Update positions of target according to mapping that was calculated
        // in init().
        for (size_t i = 0; i < mTarget->getPositions().size(); ++i)
        {
            // Calculate interpolated vertex
            Eigen::Vector interpolatedPosition = interpolate(mInterpolations[i]);
            mTarget->setPosition(i, interpolatedPosition);
        }

        break;
    }
    }

    mTarget->geometricDataChanged();
}

Vector3d MeshInterpolatorFEM::getSourcePosition(size_t targetId) const
{
    return interpolate(mInterpolations[targetId]);
}

Vector3d MeshInterpolatorFEM::interpolate(
        const MeshInterpolatorFEM::VertexInterpolation& vi) const
{
    Cell& c = mSource3->getTopology3D().getCells()[vi.mSourceCellIndex];
    Vector3d v = Vector3d::Zero();
    for (size_t i = 0; i < 4; ++i)
    {
        v += vi.mWeights[static_cast<Eigen::Index>(i)] * mSource3->getPosition(c[i]);
    }
    return v;
}

