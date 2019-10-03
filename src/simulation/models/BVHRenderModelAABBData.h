#ifndef BVHRENDERMODELAABBDATA_H
#define BVHRENDERMODELAABBDATA_H

#include "BVHRenderModelData.h"


class BVAABB;
class Polygon;
class Polygon2D;
class RenderModelManager;

class BVHRenderModelAABBData : public BVHRenderModelData
{
public:
    BVHRenderModelAABBData(
            RenderModelManager* renderModelManager,
            BVAABB* aabb,
            int level,
            bool isLeaf);

    void initialize(Renderer* renderer,
                    RenderModelManager* renderModelManager,
                    const std::shared_ptr<Polygon2D>& boxTemplate);

    // BVHRenderModelData interface
public:
    virtual void update() override;
    virtual void addToRenderer(Renderer* renderer) override;
    virtual void removeFromRenderer(Renderer* renderer) override;
    virtual void setVisible(bool visible, int level) override;
    virtual bool isVisible() const override;
    BoundingVolume::Type getType() const override;

private:
    BVAABB* mAabb;

    std::shared_ptr<Polygon> mBox;

    std::shared_ptr<PolygonRenderModel> mRenderModel;

    bool mInitialized;
};

#endif // BVHRENDERMODELAABBDATA_H
