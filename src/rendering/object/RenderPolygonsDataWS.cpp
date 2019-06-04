#include "RenderPolygonsDataWS.h"

RenderPolygonsDataWS::RenderPolygonsDataWS()
    : mPositions(GL_ARRAY_BUFFER, GL_DYNAMIC_DRAW)
    , mNormals(GL_ARRAY_BUFFER, GL_DYNAMIC_DRAW)
{

}

RenderPolygonsDataWS::RenderPolygonsDataWS(RenderPolygonsData& renderPolygonsData)
    : RenderPolygonsData (renderPolygonsData)
    , mPositions(GL_ARRAY_BUFFER, GL_DYNAMIC_DRAW)
    , mNormals(GL_ARRAY_BUFFER, GL_DYNAMIC_DRAW)
{

}

BufferedData<Eigen::Vectorf, float, 3>& RenderPolygonsDataWS::getNormalsBuffer()
{
    return mNormals;
}

BufferedData<Eigen::Vectorf, float, 3>&RenderPolygonsDataWS::getPositionsBuffer()
{
    return mPositions;
}

void RenderPolygonsDataWS::initialize()
{
    RenderPolygonsData::initialize();
    mPositions.initialize();
    mNormals.initialize();
}

void RenderPolygonsDataWS::cleanup()
{
    RenderPolygonsData::cleanup();
    mPositions.cleanup();
    mNormals.cleanup();
}

void RenderPolygonsDataWS::createBuffers()
{
    RenderPolygonsData::createBuffers();
    mPositions.createBuffer();
    mNormals.createBuffer();
}

void RenderPolygonsDataWS::refreshBuffers()
{
    RenderPolygonsData::refreshBuffers();
    mPositions.refreshBuffer();
    mNormals.refreshBuffer();
}

bool RenderPolygonsDataWS::isInitialized() const
{
    return RenderPolygonsData::isInitialized() &&
            mPositions.isInitialized() &&
            mNormals.isInitialized();
}
