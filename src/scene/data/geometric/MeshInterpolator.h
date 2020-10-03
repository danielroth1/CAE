#ifndef MESHINTERPOLATOR_H
#define MESHINTERPOLATOR_H

#include <Eigen/Core>
#include <memory>
#include <vector>

class AbstractPolygon;
class Polygon2DAccessor;

// Interpolates source to target
// mNumSourceSize - number of vertices of source
// mNumTargetSize - number of vertices of target
// numElements - number of vertices that one vertexs deformation of the target mesh
//      is depending on the deformations of the source mesh
class MeshInterpolator
{
public:

    enum class Type
    {
        MESH_MESH, FEM
    };

    MeshInterpolator(const std::shared_ptr<AbstractPolygon>& source,
                     const std::shared_ptr<AbstractPolygon>& target);

    // Solves initial system. Must be called before the first update() call.
    virtual void solve() = 0;

    virtual void update() = 0;

    virtual Type getType() const = 0;

    // Returns the interpolated position of the target vertex with the given id.
    virtual Eigen::Vector3d getInterpolatedPosition(size_t targetId) const = 0;

    std::shared_ptr<AbstractPolygon> getSource() const;

    std::shared_ptr<AbstractPolygon> getTarget() const;

    std::shared_ptr<Polygon2DAccessor> getSource2DAccessor() const;

    std::shared_ptr<Polygon2DAccessor> getTarget2DAccessor() const;

protected:
    virtual ~MeshInterpolator();

    // Fixes the position representation type (body or world space) so that
    // the one of the target is identical to the one of the source. Call this
    // method at the beginning of the update() method.
    void fixRepresentationType();

    std::shared_ptr<AbstractPolygon> mSource;
    std::shared_ptr<AbstractPolygon> mTarget;

    std::shared_ptr<Polygon2DAccessor> mSourceAccessor;
    std::shared_ptr<Polygon2DAccessor> mTargetAccessor;

};

#endif // MESHINTERPOLATOR_H
