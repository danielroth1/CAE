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

    RenderPolygonsDataWS(RenderPolygonsData& renderPolygonsData);

    BufferedData<Eigen::Vectorf, float, 3>& getPositionsBuffer();
    BufferedData<Eigen::Vectorf, float, 3>& getNormalsBuffer();

    // RenderPolygonsData interface
public:
    virtual void initialize() override;
    virtual void cleanup() override;
    virtual void createBuffers() override;
    virtual void refreshBuffers() override;
    virtual bool isInitialized() const override;

private:
    BufferedData<Eigen::Vectorf, float, 3> mPositions;
    BufferedData<Eigen::Vectorf, float, 3> mNormals;
};

#endif // RENDERPOLYGONSDATAWS_H
