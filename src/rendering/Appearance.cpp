#include "Appearance.h"
#include "RenderMaterial.h"
#include "Texture.h"

Appearance::Appearance(std::shared_ptr<Texture> texture)
    : mRenderMaterial(RenderMaterial::createFromColor({1.0f, 1.0f, 1.0f, 1.0f}))
    , mTexture(texture)
{

}

Appearance::Appearance(
        std::shared_ptr<RenderMaterial> renderMaterial,
        std::shared_ptr<Texture> texture)
    : mRenderMaterial(renderMaterial)
    , mTexture(texture)
{

}

void Appearance::bindAppearance()
{
    // enable material
    mRenderMaterial->glMaterial();

    // enable texture
    if (mTexture)
        mTexture->bind();
}

std::shared_ptr<Appearance> Appearance::createAppearanceFromColor(
        const std::array<float, 4>& color)
{
    return std::make_shared<Appearance>(
                RenderMaterial::createFromColor(color),
                nullptr);
}
