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

    Monitor<Eigen::Affine3f>& getTransform();

    // RenderPolygonsData interface
public:
    virtual void initialize();
    virtual void cleanup();
    virtual void createBuffers();
    virtual void refreshBuffers();
    virtual bool isInitialized() const;

private:
    Monitor<Eigen::Affine3f> mTransform;

};

#endif // RENDERPOLYGONSDATABS_H
