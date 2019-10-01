#ifndef BVHRENDERMODEL_H
#define BVHRENDERMODEL_H

#include <data_structures/DataStructures.h>
#include <memory>
#include <scene/model/RenderModel.h>
#include <simulation/collision_detection/broad/BVHCore.h>
#include <vector>

class BoundingVolume;
class BoundingVolumeHierarchy;
class BVHRenderModelSphereData;
class BVSphere;
class GeometricSphere;
class PolygonRenderModel;
class RenderModelManager;
class RenderPolygon2D;

// Render model for a bounding volume hierarchy.
class BVHRenderModel : public RenderModel
{
public:
    BVHRenderModel(
            RenderModelManager* renderModelManager,
            std::shared_ptr<BoundingVolumeHierarchy> bvh);

    virtual ~BVHRenderModel() override;

    // RenderModel interface
public:
    virtual void reset() override;
    virtual void update() override;
    virtual void revalidate() override;
    virtual void accept(RenderModelVisitor& v) override;
    virtual void addToRenderer(Renderer* renderer) override;
    virtual void removeFromRenderer(Renderer* renderer) override;

    // Sets all bounding volume models visible on the selected
    // render level (setRenderedLevel()). If the rendered level is
    // -1, all bounding volumes are visible. If visible is false,
    // no bouding volume will be visible independent of rendered level.
    virtual void setVisible(bool visible) override;

    // Sets the render level. Does not perform an update.
    // To see effect of the change, call update() after
    // calling this method.
    void setRenderedLevel(int renderLevel);

private:

    void addSphere(BoundingVolume* bv, size_t level, bool isLeaf);
    void addSphere(BVSphere* sphere, size_t level, bool isLeaf);

    Renderer* mRenderer;

    RenderModelManager* mRenderModelManager;

    // used to update the Spheres
    std::shared_ptr<BoundingVolumeHierarchy> mBvh;

    // Discretization resolution of the spheres. Higher values
    // mean higher resolution.
    int mSphereResolution;

    // currently rendered level
    // -1 means alls levels are rendered.
    // -2 means all leafs are rendered.
    // All other numbers render the specified level.
    int mRenderedLevel;

    // first vector: levels
    // second vector: for each level there is a number of spheres
    std::vector<
        std::vector<
            std::shared_ptr<BVHRenderModelSphereData>>> mRenderPolygons;

    std::shared_ptr<GeometricSphere> mGeometricSphereTemplate;

};

#endif // BVHRENDERMODEL_H
