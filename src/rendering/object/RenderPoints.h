#ifndef RENDERPOINTS_H
#define RENDERPOINTS_H

#include "RenderObject.h"
#include <data_structures/DataStructures.h>
#include <multi_threading/Monitor.h>

class RenderPoints : public RenderObject
{
public:
    RenderPoints();

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

    Monitor<Vectorfs>& getPoints();

private:
    Monitor<Vectorfs> mPoints;

};

#endif // RENDERPOINTS_H
