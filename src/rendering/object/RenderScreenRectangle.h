#ifndef RENDERSCREENRECTANGLE_H
#define RENDERSCREENRECTANGLE_H

#include "RenderObject.h"
#include <data_structures/DataStructures.h>

class ViewFrustum;

using namespace Eigen;

class RenderScreenRectangle : public RenderObject
{
public:
    RenderScreenRectangle(
            ViewFrustum& viewFrustum,
            const int& xStart,
            const int& yStart,
            const int& xEnd,
            const int& yEnd);

    // RenderObject interface
    void accept(RenderObjectVisitor& visitor) override;
    void draw() override;
    void drawImmediate() override;
    void drawArray() override;
    void drawVBO() override;
    void update() override;
    void createBuffers() override;
    void refreshBuffers() override;
    void cleanup() override;
    void initialize() override;

private:
    ViewFrustum& mViewFrustum;
    const int& mXStart;
    const int& mYStart;
    const int& mXEnd;
    const int& mYEnd;

};

#endif // RENDERSCREENRECTANGLE_H
