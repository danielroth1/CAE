#include "RenderPolygonsConstantDataBS.h"

RenderPolygonsConstantDataBS::RenderPolygonsConstantDataBS()
    : RenderPolygonsConstantData()
    , mPositions(GL_ARRAY_BUFFER, GL_DYNAMIC_DRAW)
    , mNormals(GL_ARRAY_BUFFER, GL_DYNAMIC_DRAW)
    , mFaces(GL_ELEMENT_ARRAY_BUFFER, GL_STATIC_DRAW)
{
}

BufferedData<Eigen::Vectorf, float, 3>&
RenderPolygonsConstantDataBS::getPositionsBuffer()
{
    return mPositions;
}

BufferedData<Eigen::Vectorf, float, 3>&
RenderPolygonsConstantDataBS::getNormalsBuffer()
{
    return mNormals;
}

BufferedData<Face, unsigned int, 3>&
RenderPolygonsConstantDataBS::getFacesBuffer()
{
    return mFaces;
}

void RenderPolygonsConstantDataBS::initialize()
{
    mPositions.initialize();
    mNormals.initialize();
    mFaces.initialize();
}

void RenderPolygonsConstantDataBS::cleanup()
{
    mPositions.cleanup();
    mNormals.cleanup();
    mFaces.cleanup();
}

void RenderPolygonsConstantDataBS::createBuffers()
{
    mPositions.createBuffer();
    mNormals.createBuffer();
    mFaces.createBuffer();
}

void RenderPolygonsConstantDataBS::refreshBuffers()
{
    mPositions.refreshBuffer();
    mNormals.refreshBuffer();
    mFaces.refreshBuffer();
}

bool RenderPolygonsConstantDataBS::isInitialized() const
{
    return mPositions.isInitialized() &&
            mNormals.isInitialized() &&
            mFaces.isInitialized();
}
