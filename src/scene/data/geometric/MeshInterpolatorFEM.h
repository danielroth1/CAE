#ifndef MESHINTERPOLATORFEM_H
#define MESHINTERPOLATORFEM_H


#include "MeshInterpolator.h"

class MeshInterpolatorFEM : public MeshInterpolator
{
public:
    MeshInterpolatorFEM(const std::shared_ptr<Polygon>& source,
                        const std::shared_ptr<Polygon>& target,
                        std::size_t numRelevantVertices);

    virtual void update() override;

private:
    void init();

    std::size_t mNumRelevantVertices;

    // Is of size mNumTargetSize, mNumRelevantVertices.
    // Stored indices are w.r.t. source vertices.
    std::vector<std::vector<std::size_t>> mRelevantSourceVertices;

    // Is of size mNumTargetSize, mNumRelevantVertices.
    std::vector<std::vector<double>> mWeights;
};

#endif // MESHINTERPOLATORFEM_H
