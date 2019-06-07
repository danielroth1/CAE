#include "SelectionRectangle.h"
#include "SelectionRectangleModel.h"

#include <rendering/object/RenderScreenRectangle.h>

#include <rendering/RenderMaterial.h>
#include <rendering/Renderer.h>


SelectionRectangleModel::SelectionRectangleModel(
        SelectionRectangle& selectionRectangle,
        ViewFrustum* viewFrustum)
    : mSelectionRectangle(selectionRectangle)
    , mViewFrustum(viewFrustum)
{
    mRenderScreenRectangle =
            std::make_shared<RenderScreenRectangle>(*mViewFrustum,
                                                    selectionRectangle.getXStart(),
                                                    selectionRectangle.getYStart(),
                                                    selectionRectangle.getXEnd(),
                                                    selectionRectangle.getYEnd());
    mRenderScreenRectangle->setRenderMaterial(
                RenderMaterial::createFromColor({1.0f, 0.0f, 0.0f, 0.8f}));
}

SelectionRectangleModel::~SelectionRectangleModel()
{

}

void SelectionRectangleModel::reset()
{
}

void SelectionRectangleModel::update()
{
    mRenderScreenRectangle->setVisible(mSelectionRectangle.isActive());
    mRenderScreenRectangle->update();
}

void SelectionRectangleModel::revalidate()
{
    // nothing to do here.
}

void SelectionRectangleModel::accept(RenderModelVisitor& /*v*/)
{
    // TODO: remove this method
}

void SelectionRectangleModel::addToRenderer(Renderer* renderer)
{
    renderer->addRenderObject(mRenderScreenRectangle);
}

void SelectionRectangleModel::removeFromRenderer(Renderer* renderer)
{
    renderer->removeRenderObject(mRenderScreenRectangle);
}
