#ifndef BVHRENDERMODELDATA_H
#define BVHRENDERMODELDATA_H


#include <memory>

#include <simulation/collision_detection/broad/BoundingVolume.h>

class BVSphere;
class GeometricSphere;
class PolygonRenderModel;
class Renderer;

class BVHRenderModelData
{
public:

    BVHRenderModelData(int level, bool isLeaf);
    virtual ~BVHRenderModelData();

    bool isToBeSetVisible(bool visible, int level);

    // Updates the data according to the bounding volume and then updates the
    // render components.
    virtual void update() = 0;

    // Adds the render components to the renderer.
    virtual void addToRenderer(Renderer* renderer) = 0;

    // Removes the render components from the renderer.
    virtual void removeFromRenderer(Renderer* renderer) = 0;

    // Sets the render components visible.
    virtual void setVisible(bool visible, int level) = 0;

    // Returns if the render components are visible.
    virtual bool isVisible() const = 0;

    virtual BoundingVolume::Type getType() const = 0;

protected:

    int mLevel;
    bool mIsLeaf;
};

#endif // BVHRENDERMODELDATA_H
