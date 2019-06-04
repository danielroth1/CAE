#ifndef RENDERPOLYGONSDATA_H
#define RENDERPOLYGONSDATA_H

#include <Eigen/Dense>

#include <memory>
#include <rendering/RenderMaterial.h>
#include <rendering/buffers/BufferedData.h>


class Texture;


// RenderPolygonsData containts rendering data that is unique for each rendered polygon.
// This includes the state of visibility and the color of the polygon.
// Shared data is represented by RenderPolygonsConstantData.
class RenderPolygonsData
{
public:

    // Initializes color to be white and visible to be true.
    RenderPolygonsData();

    RenderPolygonsData(RenderPolygonsData& rpd);

    virtual ~RenderPolygonsData();

    virtual void initialize();
    virtual void cleanup();
    virtual void createBuffers();
    virtual void refreshBuffers();
    virtual bool isInitialized() const;

    // Executes the glMaterial opengl methods for the specified material.
    // Calls the identically names method of RenderMaterial.
    void glMaterial();

    BufferedData<Eigen::Vector2f, float, 2>& getTexturesCoordinatesBufferedData();

    bool isVisible();
    void setVisible(bool visible);

    const RenderMaterial& getRenderMaterial() const;
    void setRenderMaterial(const RenderMaterial& renderMaterial);

    bool isTexturingEnabled() const;
    // Sets texturing enables if:
    //  - the texture was specified via setTexture.
    //  - the texture coordinates were specified via setTextureCoordinates.
    // If not, sets texturingEnabled false and prints a message.
    void setTexturingEnabled(bool texturingEnabled);

    // Don't use this vector to edit the texture coordinates. Use
    // setTextureCoordinates() instead.
    // Using this method, doesn't update the underlying buffer so th effect
    // won't be seen right away.
    std::vector<Eigen::Vector2f>& getTextureCoordinates();
    // Sets the given texture coordinates and informs the gpu buffers to be
    // ready for update. The update happens before the rendering of the next
    // frame.
    void setTextureCoordinates(const std::vector<Eigen::Vector2f>& tc);

    const std::shared_ptr<Texture>& getTexture() const;
    void setTexture(const std::shared_ptr<Texture>& texture);

private:
    RenderMaterial mRenderMaterial;

    std::shared_ptr<Texture> mTexture;

    BufferedData<Eigen::Vector2f, float, 2> mTextureCoordinates;

    bool mTexturingEnabled;

    bool mVisible;
};

#endif // RENDERPOLYGONSDATA_H
