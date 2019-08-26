#include "RenderObject.h"
#include <GL/glut.h>

#include <QDebug>

#include <rendering/RenderMaterial.h>

RenderObject::RenderObject()
{
    mVisible = true;
    mRenderMaterial = RenderMaterial::createFromColor({1, 1, 1, 1});
    mWireframeEnabled = false;
}

void RenderObject::setDrawMode(RenderObject::DrawMode dm)
{
    mDm = dm;
}

RenderObject::DrawMode RenderObject::getDrawMode()
{
    return mDm;
}

std::shared_ptr<RenderMaterial> RenderObject::getRenderMaterial() const
{
    return mRenderMaterial;
}

void RenderObject::setRenderMaterial(const std::shared_ptr<RenderMaterial>& renderMaterial)
{
    mRenderMaterial = renderMaterial;
}

bool RenderObject::isVisible() const
{
    return mVisible;
}

void RenderObject::setVisible(bool visible)
{
    mVisible = visible;
}

bool RenderObject::isWireframeEnabled() const
{
    return mWireframeEnabled;
}

void RenderObject::setWireframeEnabled(bool wireframeEnabled)
{
    mWireframeEnabled = wireframeEnabled;
}

Eigen::Affine3f& RenderObject::getTransform()
{
    return mTransform;
}

void RenderObject::translate(const Eigen::Vector3d& v)
{
    mTransform.translate(v.cast<float>());
}

void RenderObject::translate(const Eigen::Vector3f& v)
{
    mTransform.translate(v);
}

void RenderObject::rotate(const Eigen::Quaterniond& q)
{
    mTransform.rotate(q.cast<float>());
}

void RenderObject::rotate(const Eigen::Quaternionf& q)
{
    mTransform.rotate(q);
}

void RenderObject::scale(const Eigen::Vector3d& v)
{
    mTransform.scale(v.cast<float>());
}

void RenderObject::scale(const Eigen::Vector3f& v)
{
    mTransform.scale(v);
}

RenderObject::~RenderObject()
{

}

void RenderObject::setPolygonMode()
{
    if (mWireframeEnabled)
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    else
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}
