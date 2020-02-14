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

    // Apperance with no texture and standard material (white).
    Appearance();

    Appearance(std::shared_ptr<RenderMaterial> renderMaterial);

    // Sets white as the render material.
    Appearance(std::shared_ptr<Texture> texture);

    Appearance(
            std::shared_ptr<RenderMaterial> renderMaterial,
            std::shared_ptr<Texture> texture);

    void bindAppearance();

    static std::shared_ptr<Appearance> createDefaultAppearance();

    static std::shared_ptr<Appearance> createAppearanceFromColor(
            const std::array<float, 4>& color);

    void setName(std::string name);
    std::string getName() const;

private:
    std::string mName;
    std::shared_ptr<RenderMaterial> mRenderMaterial;
    std::shared_ptr<Texture> mTexture;
};

#endif // APPEARANCE_H
