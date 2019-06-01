#ifndef RENDERPOLYGONSDATA_H
#define RENDERPOLYGONSDATA_H

#include <Eigen/Dense>

#include <rendering/RenderMaterial.h>

// RenderPolygonsData containts rendering data that is unique for each rendered polygon.
// This includes the state of visibility and the color of the polygon.
// Shared data is represented by RenderPolygonsConstantData.
class RenderPolygonsData
{
public:

    // Initializes color to be white and visible to be true.
    RenderPolygonsData();
    virtual ~RenderPolygonsData();

    virtual void initialize() = 0;
    virtual void cleanup() = 0;
    virtual void createBuffers() = 0;
    virtual void refreshBuffers() = 0;
    virtual bool isInitialized() const = 0;

    // Executes the glMaterial opengl methods for the specified material.
    // Calls the identically names method of RenderMaterial.
    void glMaterial();

    bool isVisible();
    void setVisible(bool visible);

    const RenderMaterial& getRenderMaterial() const;
    void setRenderMaterial(const RenderMaterial& renderMaterial);

private:
    RenderMaterial mRenderMaterial;
    bool mVisible;
};

#endif // RENDERPOLYGONSDATA_H
