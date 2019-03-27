#ifndef SELECTIONRECTANGLEMODEL_H
#define SELECTIONRECTANGLEMODEL_H

#include <memory>
#include <scene/model/RenderModel.h>

class RenderScreenRectangle;
class SelectionRectangle;
class ViewFrustum;

class SelectionRectangleModel : RenderModel
{
public:

    SelectionRectangleModel(
            SelectionRectangle& selectionRectangle,
            ViewFrustum* viewFrustum);

    virtual ~SelectionRectangleModel() override;

    // RenderModel interface
public:
    virtual void reset() override;
    virtual void update() override;
    virtual void revalidate() override;
    virtual void accept(RenderModelVisitor& v) override;
    virtual void addToRenderer(Renderer* renderer) override;
    virtual void removeFromRenderer(Renderer* renderer) override;

private:
    SelectionRectangle& mSelectionRectangle;
    ViewFrustum* mViewFrustum;
    std::shared_ptr<RenderScreenRectangle> mRenderScreenRectangle;
};

#endif // SELECTIONRECTANGLEMODEL_H
