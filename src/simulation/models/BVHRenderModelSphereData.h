#ifndef BVHRENDERMODELSPHEREDATA_H
#define BVHRENDERMODELSPHEREDATA_H

#include <data_structures/DataStructures.h>

#include <memory>

class BVSphere;
class GeometricSphere;
class PolygonRenderModel;
class RenderModelManager;
class Renderer;


class BVHRenderModelSphereData
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

    ~BVHRenderModelSphereData();

    void initialize(
            Renderer* renderer,
            RenderModelManager* renderModelManager,
            GeometricSphere* geometricSphereTemplate);

    void update();

    void addToRenderer(Renderer* renderer);

    void removeFromRenderer(Renderer* renderer);

    void setVisible(bool visible, int level);

    bool isVisible() const;

    bool isToBeSetVisible(bool visible, int level);

    // Projects the vertices of the Polygon2D on the sphere
    // with the given radius.
    void setRadius(double radius);

    // Sets the position of the Polygon2D.
    void setPosition(Eigen::Vector position);

private:
    BVSphere* mBvSphere;

    std::shared_ptr<GeometricSphere> mGeometricSphere;

    std::shared_ptr<PolygonRenderModel> mRenderModel;

    double mRadius;

    double mRadiusPrevious;

    bool mIsLeaf;

    int mLevel;

    bool mInitialized;
};

#endif // BVHRENDERMODELSPHEREDATA_H
