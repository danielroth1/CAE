#ifndef RENDERPOLYGONSDATA_H
#define RENDERPOLYGONSDATA_H

#include <Eigen/Dense>

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

    Eigen::Vector4f& getColor();
    void setColor(const Eigen::Vector4f& color);

    bool isVisible();
    void setVisible(bool visible);

private:
    Eigen::Vector4f mColor; // same color as for RenderPolygons
    bool mVisible;
};

#endif // RENDERPOLYGONSDATA_H
