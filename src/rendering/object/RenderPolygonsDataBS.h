#ifndef RENDERPOLYGONSDATABS_H
#define RENDERPOLYGONSDATABS_H

#include "RenderPolygonsData.h"

#include <GL/glew.h>

#include <data_structures/DataStructures.h>
#include <multi_threading/Monitor.h>


class RenderPolygonsDataBS : public RenderPolygonsData
{
public:
    RenderPolygonsDataBS();

    RenderPolygonsDataBS(RenderPolygonsData& rpd);

    Monitor<Eigen::Affine3f>& getTransform();

    // RenderPolygonsData interface
public:
    virtual void initialize() override;
    virtual void cleanup() override;
    virtual void createBuffers() override;
    virtual void refreshBuffers() override;
    virtual bool isInitialized() const override;

private:
    Monitor<Eigen::Affine3f> mTransform;

};

#endif // RENDERPOLYGONSDATABS_H
