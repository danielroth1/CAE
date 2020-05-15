#ifndef COLLIDERRENDERMODEL_H
#define COLLIDERRENDERMODEL_H


#include <data_structures/DataStructures.h>
#include <scene/model/RenderModel.h>
#include <memory>
#include <vector>

class CollisionManager;
class RenderLines;
class RenderPoints;

class ColliderRenderModel : public RenderModel
{
public:
    ColliderRenderModel(
            const std::shared_ptr<CollisionManager>& collisionManager,
            bool renderLines,
            bool renderPoints);

    // RenderModel interface
public:
    virtual void reset() override;
    virtual void update() override;
    virtual void revalidate() override;
    virtual void accept(RenderModelVisitor& v) override;
    virtual void addToRenderer(Renderer* renderer) override;
    virtual void removeFromRenderer(Renderer* renderer) override;
    virtual void setVisible(bool visible) override;

private:

    void updateRenderCollisions(
            const std::shared_ptr<RenderLines>& renderLines,
            const std::shared_ptr<RenderPoints>& renderPoints,
            size_t startIndex,
            size_t endIndex);

    std::shared_ptr<CollisionManager> mCollisionManager;

    std::shared_ptr<RenderLines> mRenderLinesCollisions;
    std::shared_ptr<RenderLines> mRenderLinesContacts;

    std::shared_ptr<RenderPoints> mRenderPointsCollisions;
    std::shared_ptr<RenderPoints> mRenderPointsContacts;

    float mLineLength;

    bool mRenderLines;
    bool mRenderPoints;
};

#endif // COLLIDERRENDERMODEL_H
