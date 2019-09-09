#ifndef LIGHTRENDERER_H
#define LIGHTRENDERER_H

#include <Eigen/Core>

// Offers methods to control number of lights,
// positions as well as light properties
// like color, shinines, etc. of ambient, specular etc. lights.
// Is implemented in the old OpenGL way without shaders
// but with glLight commands.
class LightRenderer
{
public:
    LightRenderer();

    void initialize();

    void drawLight();

    void setLightDirection(Eigen::Vector3f lightDirection);

    Eigen::Vector3f getLightDirection();

private:
    Eigen::Vector3f mLightDirection;
};

#endif // LIGHTRENDERER_H
