#ifndef COLLIDERRENDERMODEL_H
#define COLLIDERRENDERMODEL_H


#include <data_structures/DataStructures.h>
#include <scene/model/RenderModel.h>
#include <memory>
#include <vector>

class Collider;
class RenderLines;
class RenderPoints;

class ColliderRenderModel : public RenderModel
{
public:
    ColliderRenderModel(std::shared_ptr<Collider> collider);

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
    std::shared_ptr<Collider> mCollider;

    std::shared_ptr<RenderLines> mRenderLines;

    std::shared_ptr<RenderPoints> mRenderPoints;

    float mLineLength;
};

#endif // COLLIDERRENDERMODEL_H
