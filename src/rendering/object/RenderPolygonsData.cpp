#include "RenderPolygonsData.h"

#include <iostream>

RenderPolygonsData::RenderPolygonsData()
    : mTextureCoordinates(GL_ARRAY_BUFFER, GL_STATIC_DRAW)
{
    mVisible = true;
    mTexturingEnabled = false;
}

RenderPolygonsData::RenderPolygonsData(RenderPolygonsData& rpd)
    : mRenderMaterial(rpd.mRenderMaterial)
    , mTexture(rpd.mTexture)
    , mTextureCoordinates(rpd.mTextureCoordinates)
    , mTexturingEnabled(rpd.mTexturingEnabled)
    , mVisible(rpd.mVisible)
{

}

RenderPolygonsData::~RenderPolygonsData()
{

}

void RenderPolygonsData::initialize()
{
    mTextureCoordinates.initialize();
}

void RenderPolygonsData::cleanup()
{
    mTextureCoordinates.cleanup();
}

void RenderPolygonsData::createBuffers()
{
    mTextureCoordinates.createBuffer();
}

void RenderPolygonsData::refreshBuffers()
{
    mTextureCoordinates.refreshBuffer();
}

bool RenderPolygonsData::isInitialized() const
{
    return mTextureCoordinates.isInitialized();
}

void RenderPolygonsData::glMaterial()
{
    mRenderMaterial.glMaterial();
}

BufferedData<Eigen::Vector2f, float, 2>&
RenderPolygonsData::getTexturesCoordinatesBufferedData()
{
    return mTextureCoordinates;
}

bool RenderPolygonsData::isVisible()
{
    return mVisible;
}

void RenderPolygonsData::setVisible(bool visible)
{
    mVisible = visible;
}

const RenderMaterial& RenderPolygonsData::getRenderMaterial() const
{
    return mRenderMaterial;
}

void RenderPolygonsData::setRenderMaterial(const RenderMaterial& renderMaterial)
{
    mRenderMaterial = renderMaterial;
}

bool RenderPolygonsData::isTexturingEnabled() const
{
    return mTexturingEnabled;
}

void RenderPolygonsData::setTexturingEnabled(bool texturingEnabled)
{
    if (texturingEnabled)
    {
        // Check if texturing can be enabled.

        std::string message = "";
        if (!mTexture)
            message += "Can not enable texture. Cuase: No texture.\n";
        if (mTextureCoordinates.getData().unsafe().empty())
            message += "Can not enable texture. Cuase: No texture coordinates specified.\n";

        if (message != "")
        {
            std::cout << message;
            mTexturingEnabled = false;
            return;
        }
    }
    mTexturingEnabled = texturingEnabled;
}

std::vector<Eigen::Vector2f>& RenderPolygonsData::getTextureCoordinates()
{
    return mTextureCoordinates.getData().unsafe();
}

void RenderPolygonsData::setTextureCoordinates(
        const std::vector<Eigen::Vector2f>& tc)
{
    auto textureCoordinates = mTextureCoordinates.getData().lock();
    textureCoordinates->clear();
    textureCoordinates->insert(textureCoordinates->begin(), tc.begin(), tc.end());
    mTextureCoordinates.setDataChanged(true);
}

const std::shared_ptr<Texture>& RenderPolygonsData::getTexture() const
{
    return mTexture;
}

void RenderPolygonsData::setTexture(const std::shared_ptr<Texture>& texture)
{
    mTexture = texture;
}
