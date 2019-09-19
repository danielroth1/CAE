#ifndef MESHINTERPOLATOR_H
#define MESHINTERPOLATOR_H

#include <Eigen/Core>
#include <memory>
#include <vector>

class Polygon;
class Polygon2DAccessor;

// Interpolates source to target
// mNumSourceSize - number of vertices of source
// mNumTargetSize - number of vertices of target
// numElements - number of vertices that one vertexs deformation of the target mesh
//      is depending on the deformations of the source mesh
class MeshInterpolator
{
public:
    MeshInterpolator(const std::shared_ptr<Polygon>& source,
                     const std::shared_ptr<Polygon>& target);

    // Solves initial system. Must be called before the first update() call.
    virtual void solve() = 0;

    virtual void update() = 0;

    // Returns the position of the source vertex that correspondons to the
    // target vertex with the given id.
    virtual Eigen::Vector3d getSourcePosition(size_t targetId) const = 0;

    std::shared_ptr<Polygon> getSource() const;

    std::shared_ptr<Polygon> getTarget() const;

    std::shared_ptr<Polygon2DAccessor> getSource2DAccessor() const;

    std::shared_ptr<Polygon2DAccessor> getTarget2DAccessor() const;

protected:
    virtual ~MeshInterpolator();

    std::shared_ptr<Polygon> mSource;
    std::shared_ptr<Polygon> mTarget;

    std::shared_ptr<Polygon2DAccessor> mSourceAccessor;
    std::shared_ptr<Polygon2DAccessor> mTargetAccessor;

};

#endif // MESHINTERPOLATOR_H
