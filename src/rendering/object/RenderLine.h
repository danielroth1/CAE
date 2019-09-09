#ifndef RENDERLINE_H
#define RENDERLINE_H

#include "RenderObject.h"
#include <data_structures/DataStructures.h>
#include <multi_threading/Monitor.h>

class RenderObjectVisitor;


// Renders a line. Use RenderLines to render multiple lines in a more efficient
// way.
// Disables the lightning because it's not possible to find correct vertex
// normals for lines.
class RenderLine : public RenderObject
{
public:
    RenderLine();

    // RenderObject interface
public:
    virtual void accept(RenderObjectVisitor& visitor) override;
    virtual void draw() override;
    virtual void drawImmediate() override;
    virtual void drawArray() override;
    virtual void drawVBO() override;
    virtual void update() override;
    virtual void createBuffers() override;
    virtual void refreshBuffers() override;
    virtual void cleanup() override;
    virtual void initialize() override;

    Monitor<std::vector<Eigen::Vectorf>>& getLine();

private:
    Monitor<std::vector<Eigen::Vectorf>> mLine;
};

#endif // RENDERLINE_H
