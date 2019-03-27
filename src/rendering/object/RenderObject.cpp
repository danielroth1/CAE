#include "RenderObject.h"
#include <GL/glut.h>

#include <QDebug>

RenderObject::RenderObject()
{
    mColor = Eigen::Vector4f::Ones();
    mVisible = true;
}

void RenderObject::setDrawMode(RenderObject::DrawMode dm)
{
    mDm = dm;
}

RenderObject::DrawMode RenderObject::getDrawMode()
{
    return mDm;
}

Eigen::Vector4f& RenderObject::getColor()
{
    return mColor;
}

void RenderObject::setColor(Eigen::Vector4f color)
{
    mColor = color;
}

bool RenderObject::isVisible() const
{
    return mVisible;
}

void RenderObject::setVisible(bool visible)
{
    mVisible = visible;
}

Eigen::Affine3f& RenderObject::getTransform()
{
    return mTransform;
}

RenderObject::~RenderObject()
{

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
