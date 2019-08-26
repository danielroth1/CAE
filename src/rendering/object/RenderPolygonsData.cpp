#include "RenderPolygonsData.h"

#include <iostream>

RenderPolygonsData::RenderPolygonsData()
    : mTextureCoordinates(GL_ARRAY_BUFFER, GL_DYNAMIC_DRAW)
{
    mVisible = true;
    mTexturingEnabled = false;
    mWireframeEnabled = false;
}

RenderPolygonsData::RenderPolygonsData(RenderPolygonsData& rpd)
    : mAppearances(rpd.mAppearances)
    , mTextureCoordinates(GL_ARRAY_BUFFER, GL_DYNAMIC_DRAW)
    , mTexturingEnabled(rpd.mTexturingEnabled)
    , mVisible(rpd.mVisible)
    , mWireframeEnabled(rpd.mWireframeEnabled)
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

BufferedData<Eigen::Vector2f, float, 2>&
RenderPolygonsData::getTexturesCoordinatesBufferedData()
{
    return mTextureCoordinates;
}

bool RenderPolygonsData::isVisible() const
{
    return mVisible;
}

void RenderPolygonsData::setVisible(bool visible)
{
    mVisible = visible;
}

bool RenderPolygonsData::isWireframeEnabled() const
{
    return mWireframeEnabled;
}

void RenderPolygonsData::setWireframeEnabled(bool wireframeEnabled)
{
    mWireframeEnabled = wireframeEnabled;
}

void RenderPolygonsData::setAppearances(
        const std::shared_ptr<Appearances>& appearances)
{
    mAppearances = appearances;
}

std::shared_ptr<Appearances> RenderPolygonsData::getAppearances() const
{
    return mAppearances;
}

bool RenderPolygonsData::isTexturingEnabled() const
{
    return mTexturingEnabled;
}

void RenderPolygonsData::setTexturingEnabled(bool texturingEnabled)
{
    if (texturingEnabled)
    {
        std::string message = "";
        if (!mAppearances)
            message += "Can not enable texturing. Cuase: No appearance.\n";

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

void RenderPolygonsData::setPolygonMode() const
{
    if (mWireframeEnabled)
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    else
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}
