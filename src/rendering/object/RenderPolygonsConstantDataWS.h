#ifndef RENDERPOLYGONSCONSTANTDATAWS_H
#define RENDERPOLYGONSCONSTANTDATAWS_H

#include "RenderPolygonsConstantData.h"

#include <GL/glew.h>

#include <data_structures/DataStructures.h>
#include <multi_threading/Monitor.h>
#include <rendering/buffers/BufferedData.h>


class RenderPolygonsConstantDataWS : public RenderPolygonsConstantData
{
public:
    RenderPolygonsConstantDataWS();

    BufferedData<Face, unsigned int, 3>& getFacesBuffer();

private:
    BufferedData<Face, unsigned int, 3> mFaces;

    // RenderPolygonsConstantData interface
public:
    virtual void initialize();
    virtual void cleanup();
    virtual void createBuffers();
    virtual void refreshBuffers();
    virtual bool isInitialized() const;
};

#endif // RENDERPOLYGONSCONSTANTDATAWS_H
