#include "RenderModel.h"

#include <rendering/Renderer.h>

bool RenderModel::isAddedToRenderer()
{
    return mAddedToRenderer;
}

void RenderModel::setAddedToRenderer(bool addedToRenderer)
{
    mAddedToRenderer = addedToRenderer;
}

bool RenderModel::isAlwaysUpdate()
{
    return mAlwaysUpdate;
}

void RenderModel::setAlwaysUpdate(bool alwaysUpdate)
{
    mAlwaysUpdate = alwaysUpdate;
}

std::shared_ptr<Appearances> RenderModel::getAppearances()
{
    return mAppearances;
}

void RenderModel::setAppearances(const std::shared_ptr<Appearances>& appearances)
{
    if (!mAppearances ||
        getRenderedAppearances() == nullptr ||
        getRenderedAppearances() == mAppearances)
    {
        setRenderedAppearances(appearances);
    }

    mAppearances = appearances;
}

std::shared_ptr<Appearances> RenderModel::getRenderedAppearances()
{
    return nullptr;
}

void RenderModel::setRenderedAppearances(
        const std::shared_ptr<Appearances>& /*appearances*/)
{
}

bool RenderModel::isVisible() const
{
    return mVisible;
}

void RenderModel::setVisible(bool visible)
{
    mVisible = visible;
}

bool RenderModel::isWireframeEnabled() const
{
    return mWireframeEnabled;
}

void RenderModel::setWireframeEnabled(bool wireframeEnabled)
{
    mWireframeEnabled = wireframeEnabled;
}

RenderModel::RenderModel()
{
    mAddedToRenderer = false;
    mAlwaysUpdate = false;
    mVisible = true;
    mWireframeEnabled = false;
}

RenderModel::~RenderModel()
{

}
