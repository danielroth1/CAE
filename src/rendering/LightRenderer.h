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

    void setLightPosition(Eigen::Vector3f lightPosition);

    Eigen::Vector3f getLightPosition();

private:
    Eigen::Vector3f mLightPosition;
};

#endif // LIGHTRENDERER_H
