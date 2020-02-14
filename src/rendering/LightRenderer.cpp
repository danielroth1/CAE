#include "LightRenderer.h"

#include <GL/glut.h>
#include <GL/glu.h>

#include <iostream>

LightRenderer::LightRenderer()
{
    mLightDirection = Eigen::Vector3f(1.0f, 0.0f, 0.0f);
    mLightPosition = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    mLightPositionSet = false;
}

void LightRenderer::initialize()
{
    // set lighting
    GLfloat global_ambient[] = { 0.2, 0.2, 0.2, 1 };
    GLfloat ambientLight[] =   { 0.1, 0.1, 0.1, 1 };
    GLfloat diffuseLight[] =   { 0.9, 0.9, 0.9, 1 };
    GLfloat specularLight[] =  { 1, 1, 1, 1 };
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, global_ambient);
    glLightfv(GL_LIGHT0, GL_AMBIENT, ambientLight);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuseLight);
    glLightfv(GL_LIGHT0, GL_SPECULAR, specularLight);
    glEnable(GL_LIGHT0);

    // enable use of glColor instead of glMaterial for ambient and diffuse property
//    glEnable(GL_COLOR_MATERIAL);
//    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
}

void LightRenderer::drawLight()
{
    // set light position in within current coordinate system
    if (mLightPositionSet)
    {
        GLfloat lp[] = { mLightPosition(0),
                         mLightPosition(1),
                         mLightPosition(2), 1.0f };
        glLightfv(GL_LIGHT0, GL_POSITION, lp);
    }
    else
    {
        GLfloat lp[] = { -mLightDirection(0),
                         -mLightDirection(1),
                         -mLightDirection(2), 0.0f };
        glLightfv(GL_LIGHT0, GL_POSITION, lp);
    }

    glEnable(GL_LIGHTING);
}

void LightRenderer::setLightDirection(Eigen::Vector3f lightDirection)
{
    mLightDirection = lightDirection;
    mLightPositionSet = false;
}

void LightRenderer::setLightPosition(const Eigen::Vector3f& lightPosition)
{
    mLightPosition = lightPosition;
    mLightPositionSet = true;
}

Eigen::Vector3f LightRenderer::getLightDirection()
{
    return mLightDirection;
}
