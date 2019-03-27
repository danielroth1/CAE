#ifndef LINEARFORCERENDERMODEL_H
#define LINEARFORCERENDERMODEL_H

#include <data_structures/DataStructures.h>
#include <scene/model/RenderModel.h>

class LinearForce;
class RenderLine;

// This class uses the draw line capabilities of the
// renderer to render linear forces
// Uses the RenderObject RenderLine to interact with renderer
class LinearForceRenderModel : public RenderModel
{
public:
    LinearForceRenderModel(std::shared_ptr<LinearForce> linearForce);

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
    std::shared_ptr<LinearForce> mLinearForce;

    std::shared_ptr<RenderLine> mRenderLine;
};

#endif // LINEARFORCERENDERMODEL_H
