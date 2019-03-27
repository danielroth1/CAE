#ifndef RENDERLINES_H
#define RENDERLINES_H

#include "RenderObject.h"

#include <multi_threading/Monitor.h>



class RenderLines : public RenderObject
{
public:
    RenderLines();

    // RenderObject interface
public:
    virtual void accept(RenderObjectVisitor& visitor);
    virtual void draw();
    virtual void drawImmediate();
    virtual void drawArray();
    virtual void drawVBO();

    // Call this method whenever the number of lines changes.
    virtual void update();
    virtual void createBuffers();
    virtual void refreshBuffers();
    virtual void cleanup();
    virtual void initialize();

    Monitor<Vectorfs>& getLines();
    Monitor<std::vector<std::array<unsigned int, 2>>>& getIndices();

private:
    // Each two vertices i, i+1 are start and end point
    // of a lines. The lines are not connected.
    Monitor<Vectorfs> mLines;

    Monitor<std::vector<std::array<unsigned int, 2>>> mIndices;
};

#endif // RENDERLINES_H
