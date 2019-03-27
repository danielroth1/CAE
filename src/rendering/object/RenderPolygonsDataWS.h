#ifndef RENDERPOLYGONSDATAWS_H
#define RENDERPOLYGONSDATAWS_H

#include "RenderPolygonsData.h"

#include <GL/glew.h>

#include <data_structures/DataStructures.h>
#include <multi_threading/Monitor.h>
#include <rendering/buffers/BufferedData.h>


class RenderPolygonsDataWS : public RenderPolygonsData
{
public:
    RenderPolygonsDataWS();

    BufferedData<Eigen::Vectorf, float, 3>& getPositionsBuffer();
    BufferedData<Eigen::Vectorf, float, 3>& getNormalsBuffer();

    // RenderPolygonsData interface
public:
    virtual void initialize();
    virtual void cleanup();
    virtual void createBuffers();
    virtual void refreshBuffers();
    virtual bool isInitialized() const;

private:
    BufferedData<Eigen::Vectorf, float, 3> mPositions;
    BufferedData<Eigen::Vectorf, float, 3> mNormals;
};

#endif // RENDERPOLYGONSDATAWS_H
