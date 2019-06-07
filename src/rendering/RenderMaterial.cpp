#include "RenderMaterial.h"

#include <GL/glut.h>
#include <GL/glu.h>

#include <cmath>


RenderMaterial::RenderMaterial(
        const std::array<float, 4>& ambient,
        const std::array<float, 4>& diffuse,
        const std::array<float, 4>& specular,
        float shininess)
    : mAmbient(ambient)
    , mDiffuse(diffuse)
    , mSpecular(specular)
    , mShininess(shininess)
{
    mAmbientAndDiffuseIdentical = true;
    for (size_t i = 0; i < 4; ++i)
    {
        if (std::abs(ambient[i] - diffuse[i]) > 1e-5f)
            mAmbientAndDiffuseIdentical = false;
    }

}

void RenderMaterial::glMaterial()
{
    if (mAmbientAndDiffuseIdentical)
    {
        glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, mAmbient.data());
    }
    else
    {
        glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, mAmbient.data());
        glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, mDiffuse.data());
    }
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, mShininess);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mSpecular.data());
}

std::array<float, 4> RenderMaterial::getAmbientDefault()
{
    return {0.2f, 0.2f, 0.2f, 1.0f};
}

std::array<float, 4> RenderMaterial::getSpecularDefault()
{
    return {0.8f, 0.8f, 0.8f, 1.0f};
}

std::array<float, 4> RenderMaterial::getDiffuseDefault()
{
    return {0.0f, 0.0f, 0.0f, 1.0f};
}

float RenderMaterial::getShininessDefault()
{
    return 0.0f;
}

std::shared_ptr<RenderMaterial> RenderMaterial::createFromColor(
        const std::array<float, 4>& color)
{
    return std::make_shared<RenderMaterial>(
                color, color, getSpecularDefault(), getShininessDefault());
}
