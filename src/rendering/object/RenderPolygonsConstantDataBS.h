#ifndef RENDERPOLYGONSCONSTANTDATABS_H
#define RENDERPOLYGONSCONSTANTDATABS_H

#include "RenderPolygonsConstantData.h"

#include <GL/glew.h>

#include <data_structures/DataStructures.h>
#include <multi_threading/Monitor.h>
#include <rendering/buffers/BufferedData.h>


class RenderPolygonsConstantDataBS : public RenderPolygonsConstantData
{
public:
    RenderPolygonsConstantDataBS();

    BufferedData<Eigen::Vectorf, float, 3>& getPositionsBuffer();
    BufferedData<Eigen::Vectorf, float, 3>& getNormalsBuffer();
    BufferedData<Face, unsigned int, 3>& getFacesBuffer();

    // RenderPolygonsConstantData interface
public:
    virtual void initialize();
    virtual void cleanup();
    virtual void createBuffers();
    virtual void refreshBuffers();
    virtual bool isInitialized() const;

private:
    BufferedData<Eigen::Vectorf, float, 3> mPositions;
    BufferedData<Eigen::Vectorf, float, 3> mNormals;
    BufferedData<Face, unsigned int, 3> mFaces;

};

#endif // RENDERPOLYGONSCONSTANTDATABS_H
