#ifndef RENDERLINES_H
#define RENDERLINES_H

#include "RenderObject.h"

#include <multi_threading/Monitor.h>


// Renders lines. Disables the lightning because it's not possible to find
// correct vertex normals for lines.
// Needs to update lines and indicies. Use the corresponding getters for that.
//
// - To render a line that follows the points in lines, fill indices like this
//   indices = {0, 1, 1, 2, 2, ..., nPoints-1, nPoints-1}
// - To render lines whichs points are stored after each other in the lines vector
//   independently from each other, store the indicies like this:
//   indicies = {0, 1, 2, 3, ..., nLines * 2 - 1}
//   Access each lines start point with [ i * 2 ] and end point with [ i * 2 + 1 ]
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
