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

MeshInterpolatorFEM::~MeshInterpolatorFEM()
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
            Cell& c = mSource3->getTopology3D().getCellIds()[j];

            bary = calculateBaryzentricCoordinates(c, p);

            VertexInterpolation vi(bary, j, i);
            if (-wt < bary[0] && bary[0] < 1 + wt &&
                -wt < bary[1] && bary[1] < 1 + wt &&
                -wt < bary[2] && bary[2] < 1 + wt &&
                -wt < bary[3] && bary[3] < 1 + wt)
            {
                candidateInterpolations.push_back(vi);
            }
        }

        bool candidateFound = false;

        if (candidateInterpolations.empty())
        {
            // The point lies outside. Search for the closests outside triangle.
            // Then calculate the baryzentric coordinates for the corresponding
            // tetrahedron and use those.
            // Visual artifacts are limitted if the point is close enough to
            // the tetrahedron.

            // obtain index of closest triangle:
            // triangle_index, distance
            std::vector<std::tuple<size_t, double>> tuples;

            std::vector<TopologyFace>& outerFaces =
                    mSource3->getTopology3D().getOuterTopology().getFaces();
            for (size_t j = 0; j < outerFaces.size(); ++j)
            {
//                TopologyFace& f = outerFaces[j];

                // If this outer triangle is not bound to a cell, ignore it.
                Polygon3DTopology& pt = mSource3->getTopology3D();
                ID f3D = pt.to3DFaceIndex(j);
                TopologyFace& f3d = pt.getFace(f3D);
                if (f3d.getCellIds().empty())
                    continue;

                std::array<Vector, 3> v; // vertices
                v[0] = mSource3->getPosition(f3d.getVertexIds()[0]);
                v[1] = mSource3->getPosition(f3d.getVertexIds()[1]);
                v[2] = mSource3->getPosition(f3d.getVertexIds()[2]);

                Eigen::Vector3d bary = calculateBaryzentricCoordinates(f3d, p);

                Eigen::Vector pTriangle =
                        bary(0) * v[0] + bary(1) * v[1] + bary(2) * v[2];
                double distance = (pTriangle - p).norm();
                tuples.push_back(std::make_tuple(j, distance));

            }

            std::sort(tuples.begin(), tuples.end(),
                      [](const std::tuple<size_t, double>& t1,
                      const std::tuple<size_t, double>& t2)
            {
                return std::get<1>(t1) < std::get<1>(t2);
            });

            if (!tuples.empty())
            {
                candidateFound = true;

                size_t closestTriangleIndex = std::get<0>(tuples[0]);
                double distance = std::get<1>(tuples[0]);

                // we have a face if of the outer face mesh
                // we need to convert it to the one of the corresponding inner mesh
                // sth. like Polygon3DTopology::getOuterFaceIds()
                // similar to Polygon3DTopology::getOuterVertexIds()

                Polygon3DTopology& pt = mSource3->getTopology3D();
                ID f3D = pt.to3DFaceIndex(closestTriangleIndex);
                TopologyFace& f = pt.getFace(f3D);

                if (printErrors && f.getCellIds().empty())
                {
                    std::cerr << "Error: No adjacent cell found for outer face.\n";
                }
                else
                {
                    if (f.getCellIds().size() > 1)
                    {
                        std::cout << "Warning: Outer face has multiple adjacent "
                                     "cells. One cell is chosen arbitrarily.\n";
                    }

                    ID cellId = f.getCellIds()[0];
//                    Cell& c = mSource3->getTopology3D().getOuterTopology().getCellIds()[cellId];
                    Cell& c = mSource3->getTopology3D().getCellIds()[cellId];

                    bary = calculateBaryzentricCoordinates(c, p);
                    VertexInterpolation vi(bary, cellId, i);
                    candidateInterpolations.push_back(vi);
                }
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
            candidateFound = true;

//            std::cout << "Adds interpolation with "
//                      << "weights = "
//                      << vi.mWeights(0) << ", " << vi.mWeights(1) << ", "
//                      << vi.mWeights(2) << ", " << vi.mWeights(3) << "\n";
        }

        if (printErrors && !candidateFound)
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

    fixRepresentationType();

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

    mTarget->update();
}

Vector3d MeshInterpolatorFEM::getSourcePosition(size_t targetId) const
{
    return interpolate(mInterpolations[targetId]);
}

Vector3d MeshInterpolatorFEM::interpolate(
        const MeshInterpolatorFEM::VertexInterpolation& vi) const
{
    Cell& c = mSource3->getTopology3D().getCellIds()[vi.mSourceCellIndex];
    Vector3d v = Vector3d::Zero();
    for (size_t i = 0; i < 4; ++i)
    {
        v += vi.mWeights[static_cast<Eigen::Index>(i)] * mSource3->getPosition(c[i]);
    }
    return v;
}

Vector4d MeshInterpolatorFEM::calculateBaryzentricCoordinates(
        const Cell& c, const Eigen::Vector& p) const
{
    std::array<Vector, 4> v; // vertices
    Eigen::Vector4d bary; // baryzentric coordinatees

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

    Vector center = 0.25 * (v[0] + v[1] + v[2] + v[3]);

    bary[0] = 0.25 + (p - center).dot(r4.cross(r5) / J);
    bary[1] = 0.25 + (p - center).dot(r2.cross(r3) / J);
    bary[2] = 0.25 + (p - center).dot(r3.cross(r1) / J);
    bary[3] = 0.25 + (p - center).dot(r1.cross(r2) / J);

    return bary;
}

Vector3d MeshInterpolatorFEM::calculateBaryzentricCoordinates(
        const TopologyFace& f, const Vector& p) const
{

    std::array<Vector, 3> v; // vertices

    // Vertex positions
    v[0] = mSource3->getPosition(f.getVertexIds()[0]);
    v[1] = mSource3->getPosition(f.getVertexIds()[1]);
    v[2] = mSource3->getPosition(f.getVertexIds()[2]);

    Eigen::Vector3d v13 = v[9] - v[2];
    Eigen::Vector3d v23 = v[1] - v[2];
    double temp = v13.dot(v23);

    Eigen::Matrix2d A;
    A << v13.dot(v13), temp,
            temp, v23.dot(v23);
    Eigen::Vector2d b;
    b << v13.dot(p - v[2]),
            v23.dot(p - v[2]);

    Eigen::Vector2d bary2 = A.inverse() * b;
    Eigen::Vector3d bary = Eigen::Vector3d(
                bary2(0), bary2(1), 1 - bary2(0) - bary2(1));

    if (bary(0) < 0.0)
    {
        Vector3d d = v[2] - v[1];
        double d2 = d.dot(d);
        double t = (d2 < 1e-10) ? 0.5 : d.dot(p - v[2]) / d2;
        t = std::max(0.0, std::min(1.0, t));
        bary(0) = 0.0;
        bary(1) = 1.0 - t;
        bary(2) = t;
    }
    else if (bary(1) < 0.0)
    {
        Vector3d d = v[0] - v[2];
        double d2 = d.dot(d);
        double t = (d2 < 1e-10) ? 0.5 : d.dot(p - v[0]) / d2;
        t = std::max(0.0, std::min(1.0, t));
        bary(1) = 0.0;
        bary(2) = 1.0 - t;
        bary(0) = t;
    }
    else if (bary(2) < 0.0)
    {
        Vector3d d = v[1] - v[0];
        double d2 = d.dot(d);
        double t = (d2 < 1e-10) ? 0.5 : d.dot(p - v[1]) / d2;
        t = std::max(0.0, std::min(1.0, t));
        bary(2) = 0.0;
        bary(0) = 1.0 - t;
        bary(1) = t;
    }

    return bary;
}

