#include "MeshInterpolatorFEM.h"
#include "Polygon.h"

#include <algorithm>
#include <iostream>

MeshInterpolatorFEM::MeshInterpolatorFEM(
        const std::shared_ptr<Polygon>& source,
        const std::shared_ptr<Polygon>& target,
        std::size_t numRelevantVertices)
    : MeshInterpolator (source, target)
    , mNumRelevantVertices(numRelevantVertices)
{
    init();
}

void MeshInterpolatorFEM::update()
{
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
            Eigen::Vector interpolatedPosition = Eigen::Vector::Zero();

            for (size_t j = 0; j < mNumRelevantVertices; ++j)
            {
                interpolatedPosition +=
                        mWeights[i][j] * mSource->getPosition(mRelevantSourceVertices[i][j]);
            }

            mTarget->setPosition(i, interpolatedPosition);
        }

        break;
    }
    }

}

void MeshInterpolatorFEM::init()
{
    mRelevantSourceVertices.clear();
    mWeights.clear();

    mRelevantSourceVertices.reserve(mTarget->getSize());
    mWeights.reserve(mTarget->getSize());

    // Fill mRelevantSourceVertices.
    for (std::size_t i = 0; i < mTarget->getSize(); ++i)
    {
        Eigen::Vector targetPos = mTarget->getPosition(i);

        std::vector<std::pair<double, std::size_t>> distIndexPairs;

        // Gather the closest mNumRelevantVertices.
        for (std::size_t j = 0; j < mSource->getSize(); ++j)
        {
            Eigen::Vector sourcePos = mSource->getPosition(i);
            Eigen::Vector dir = targetPos - sourcePos;
            double distance = dir.norm();
            distIndexPairs.push_back(std::make_pair(distance, j));
        }

        // Sort distIndexPairs so that indices whose vertices have the smallest
        // distance are at the start of the vector.
        std::sort(distIndexPairs.begin(),
                  distIndexPairs.end(),
                  [](const std::pair<double, std::size_t>& p1,
                  const std::pair<double, std::size_t>& p2)
        {
            return p1.first < p2.first;
        });

        // Extract the first mNumRelevantVertices.
        std::vector<std::size_t> relevantVertexIndices;
        relevantVertexIndices.reserve(mNumRelevantVertices);
        for (std::size_t j = 0; j < mNumRelevantVertices; ++j)
        {
            relevantVertexIndices.push_back(distIndexPairs[j].second);
        }
        mRelevantSourceVertices.push_back(relevantVertexIndices);

        // Insert the normalized weights of the first mNumRelevantVertices.
        // By using normalized weights, a simple linear kernel is applied.
        std::vector<double> weights;
        weights.reserve(mNumRelevantVertices);
        double totalWeight = 0.0;
        for (std::size_t j = 0; j < mNumRelevantVertices; ++j)
        {
            totalWeight += distIndexPairs[i].first;
        }
        for (std::size_t j = 0; j < mNumRelevantVertices; ++j)
        {
            weights.push_back(distIndexPairs[i].first / totalWeight);
        }
    }
}
