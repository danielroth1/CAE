#ifndef INTERPOLATIONFEMDEMO_H
#define INTERPOLATIONFEMDEMO_H

#include <modules/demo_loader/Demo.h>
#include <scene/data/geometric/MeshInterpolator.h>
#include <scene/scene_graph/SGControl.h>

class ApplicationControl;
class MeshInterpolatorFEM;
class MeshInterpolatorRenderModel;


class InterpolationFEMDemo : public Demo
{
public:
    // \param simpleMeshes - if true, a sphere is interpolated by a box.
    //      if false, advanced_crew_escape_suit is interpolated by
    //      advanced_crew_escape_suit_convex.
    InterpolationFEMDemo(
            ApplicationControl* ac,
            bool simpleMeshes,
            MeshInterpolator::Type interpolatorType);

    // Demo interface
public:
    virtual std::string getName();
    virtual void load();
    virtual void unload();

private:

    ApplicationControl* mAc;
    MeshInterpolator::Type mInterpolatorType;
    bool mSimpleMeshes;
};

#endif // INTERPOLATIONFEMDEMO_H
