#include "RenderPolygonsDataBS.h"

RenderPolygonsDataBS::RenderPolygonsDataBS()
{

}

RenderPolygonsDataBS::RenderPolygonsDataBS(RenderPolygonsData& rpd)
    : RenderPolygonsData (rpd)
{

}

Monitor<Eigen::Affine3f>& RenderPolygonsDataBS::getTransform()
{
    return mTransform;
}

void RenderPolygonsDataBS::initialize()
{
    RenderPolygonsData::initialize();
}

void RenderPolygonsDataBS::cleanup()
{
    RenderPolygonsData::cleanup();
}

void RenderPolygonsDataBS::createBuffers()
{
    RenderPolygonsData::createBuffers();
}

void RenderPolygonsDataBS::refreshBuffers()
{
    RenderPolygonsData::refreshBuffers();
}

bool RenderPolygonsDataBS::isInitialized() const
{
    return RenderPolygonsData::isInitialized();
}
