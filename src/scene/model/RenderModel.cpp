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
