#include "RenderPolygonsData.h"

RenderPolygonsData::RenderPolygonsData()
{
    mVisible = true;
}

RenderPolygonsData::~RenderPolygonsData()
{

}

void RenderPolygonsData::glMaterial()
{
    mRenderMaterial.glMaterial();
}

bool RenderPolygonsData::isVisible()
{
    return mVisible;
}

void RenderPolygonsData::setVisible(bool visible)
{
    mVisible = visible;
}

const RenderMaterial& RenderPolygonsData::getRenderMaterial() const
{
    return mRenderMaterial;
}

void RenderPolygonsData::setRenderMaterial(const RenderMaterial& renderMaterial)
{
    mRenderMaterial = renderMaterial;
}
