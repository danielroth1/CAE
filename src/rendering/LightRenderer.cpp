#include "LightRenderer.h"

#include <GL/glut.h>
#include <GL/glu.h>

#include <iostream>

LightRenderer::LightRenderer()
{
//    mLightDirection = Eigen::Vector3f(1.7f, 1.3f, 1.7f);
    mLightDirection = Eigen::Vector3f(1.0f, 0.0f, 0.0f);
}

void LightRenderer::initialize()
{
    // set lighting and material
    float factorGlobal = 0.0f;
    float factorAmbient = 0.30f;
    float factorDiffuse = 0.55f;
    float factorSpecular = 0.15f;
    GLfloat global_ambient[] = { factorGlobal, factorGlobal, factorGlobal, factorGlobal };
    GLfloat ambientLight[] =   { factorAmbient, factorAmbient, factorAmbient, factorAmbient };
    GLfloat diffuseLight[] =   { factorDiffuse, factorDiffuse, factorDiffuse, factorDiffuse };
    GLfloat specularLight[] =  { factorSpecular, factorSpecular, factorSpecular, factorSpecular };
    GLfloat shininess = 0.0;//128.0f;
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, global_ambient);
    glLightfv(GL_LIGHT0, GL_AMBIENT, ambientLight);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuseLight);
    glLightfv(GL_LIGHT0, GL_SPECULAR, specularLight);
    glLightf(GL_LIGHT0, GL_SHININESS, shininess);
    glEnable(GL_LIGHT0);
    // enable use of glColor instead of glMaterial for ambient and diffuse property
//    glEnable(GL_COLOR_MATERIAL);
//    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    // white shiny specular highlights
    GLfloat specularLightMaterial[] =  { 1.0f, 1.0f, 1.0f, 1.0f };
    GLfloat shininessMaterial = 128.0f;
    glMaterialf(GL_FRONT_AND_BACK,GL_SHININESS,shininessMaterial);
    glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR, specularLightMaterial);
}

void LightRenderer::drawLight()
{
    // set light position in within current coordinate system
    GLfloat lp[] = { -mLightDirection(0),
                     -mLightDirection(1),
                     -mLightDirection(2), 0.0f };
    glLightfv(GL_LIGHT0, GL_POSITION, lp);

//    glDisable(GL_LIGHTING);
    // draw yellow sphere for light source
//    glPushMatrix();
//    glTranslatef(lp[0], lp[1], lp[2]);
//    glColor3f(1.0f, 1.0f, 0.0f);
//    float color[4] = {1.0f, 1.0f, 0.0f, 1.0f};
//    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color);
//    glutSolidSphere(0.05, 64, 64);
//    glPopMatrix();

    glEnable(GL_LIGHTING);
}

void LightRenderer::setLightDirection(Eigen::Vector3f lightDirection)
{
    mLightDirection = lightDirection;
}

Eigen::Vector3f LightRenderer::getLightDirection()
{
    return mLightDirection;
}
