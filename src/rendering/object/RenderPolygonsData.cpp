#include "RenderPolygonsData.h"

RenderPolygonsData::RenderPolygonsData()
{
    mColor = Eigen::Vector4f::Ones();
    mVisible = true;
}

RenderPolygonsData::~RenderPolygonsData()
{

}

Eigen::Vector4f& RenderPolygonsData::getColor()
{
    return mColor;
}

void RenderPolygonsData::setColor(const Eigen::Vector4f& color)
{
    mColor = color;
}

bool RenderPolygonsData::isVisible()
{
    return mVisible;
}

void RenderPolygonsData::setVisible(bool visible)
{
    mVisible = visible;
}
