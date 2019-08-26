#ifndef MESHINTERPOLATORFEM_H
#define MESHINTERPOLATORFEM_H


#include "MeshInterpolator.h"
#include <data_structures/DataStructures.h>

class Polygon3D;
class TopologyFace;
class VertexInterpolation;

class MeshInterpolatorFEM : public MeshInterpolator
{
public:
    MeshInterpolatorFEM(const std::shared_ptr<Polygon3D>& source,
                        const std::shared_ptr<Polygon>& target);

    virtual ~MeshInterpolatorFEM() override;

    void solve();

    // MeshInterpolator interface
public:
    // Calls geometricDataChanged() of target.
    virtual void update() override;
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

    Eigen::Vector3d interpolate(const VertexInterpolation& vi) const;

    Eigen::Vector4d calculateBaryzentricCoordinates(
            const Cell& c, const Eigen::Vector& p) const;

    Eigen::Vector3d calculateBaryzentricCoordinates(
            const TopologyFace& f, const Eigen::Vector& p) const;

    std::shared_ptr<Polygon3D> mSource3;

    bool mSolved;

    // Baryzentric cooridnates w.r.t. the low res mesh.
    // There is one per point of the high res mesh.
    std::vector<VertexInterpolation> mInterpolations;
};

#endif // MESHINTERPOLATORFEM_H
