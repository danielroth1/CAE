#ifndef APPEARANCE_H
#define APPEARANCE_H

#include <Eigen/Core>
#include <memory>
#include <rendering/buffers/BufferedData.h>

class RenderMaterial;
class Texture;

class Appearance
{
public:
    // Sets white as the render material.
    Appearance(std::shared_ptr<Texture> texture);

    Appearance(
            std::shared_ptr<RenderMaterial> renderMaterial,
            std::shared_ptr<Texture> texture);

    void bindAppearance();

    static std::shared_ptr<Appearance> createAppearanceFromColor(
            const std::array<float, 4>& color);

private:
    std::shared_ptr<RenderMaterial> mRenderMaterial;
    std::shared_ptr<Texture> mTexture;
};

#endif // APPEARANCE_H
