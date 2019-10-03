#ifndef BVHRENDERMODELSPHEREDATA_H
#define BVHRENDERMODELSPHEREDATA_H

#include "BVHRenderModelData.h"

#include <data_structures/DataStructures.h>

#include <memory>

class BVSphere;
class GeometricSphere;
class PolygonRenderModel;
class RenderModelManager;
class Renderer;


class BVHRenderModelSphereData : public BVHRenderModelData
{
public:
    BVHRenderModelSphereData();

    BVHRenderModelSphereData(
            RenderModelManager* renderModelManager,
            BVSphere* bvSphere,
            int level,
            double radius,
            int resolution,
            bool isLeaf);

    // Does not recreate the sphere polygon. Instead uses the provided
    // template. This is a lot faster.

    // TODO: SphereData is hold multiple times
    // by different shared pointers?
    // error somewhere in destructor
    BVHRenderModelSphereData(
            RenderModelManager* renderModelManager,
            GeometricSphere* geometricSphereTemplate,
            BVSphere* bvSphere,
            int level,
            double radius,
            bool isLeaf);

    virtual ~BVHRenderModelSphereData() override;

    void initialize(
            Renderer* renderer,
            RenderModelManager* renderModelManager,
            GeometricSphere* geometricSphereTemplate);

    // Projects the vertices of the Polygon2D on the sphere
    // with the given radius.
    void setRadius(double radius);

    // Sets the position of the Polygon2D.
    void setPosition(Eigen::Vector position);

    // BVHRenderModelData interface
public:
    void update() override;

    void addToRenderer(Renderer* renderer) override;

    void removeFromRenderer(Renderer* renderer) override;

    void setVisible(bool visible, int level) override;

    bool isVisible() const override;

    BoundingVolume::Type getType() const override;

private:
    BVSphere* mBvSphere;

    std::shared_ptr<GeometricSphere> mGeometricSphere;

    std::shared_ptr<PolygonRenderModel> mRenderModel;

    double mRadius;

    double mRadiusPrevious;

    bool mInitialized;
};

#endif // BVHRENDERMODELSPHEREDATA_H
