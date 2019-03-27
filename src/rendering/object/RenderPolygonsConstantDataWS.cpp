#include "RenderPolygonsConstantDataWS.h"

RenderPolygonsConstantDataWS::RenderPolygonsConstantDataWS()
    : mFaces(GL_ELEMENT_ARRAY_BUFFER, GL_DYNAMIC_DRAW)
{

}

BufferedData<Face, unsigned int, 3>& RenderPolygonsConstantDataWS::getFacesBuffer()
{
    return mFaces;
}

void RenderPolygonsConstantDataWS::initialize()
{
    mFaces.initialize();
}

void RenderPolygonsConstantDataWS::cleanup()
{
    mFaces.cleanup();
}

void RenderPolygonsConstantDataWS::createBuffers()
{
    mFaces.createBuffer();
}

void RenderPolygonsConstantDataWS::refreshBuffers()
{
    mFaces.refreshBuffer();
}

bool RenderPolygonsConstantDataWS::isInitialized() const
{
    return mFaces.isInitialized();
}
