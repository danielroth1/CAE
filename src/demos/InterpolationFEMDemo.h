#ifndef INTERPOLATIONFEMDEMO_H
#define INTERPOLATIONFEMDEMO_H

#include <modules/demo_loader/Demo.h>
#include <scene/scene_graph/SGControl.h>

class ApplicationControl;
class MeshInterpolatorFEM;
class MeshInterpolatorRenderModel;


class InterpolationFEMDemo : public Demo
{
public:
    InterpolationFEMDemo(ApplicationControl* ac);

    // Demo interface
public:
    virtual std::string getName();
    virtual void load();
    virtual void unload();

private:

    void addInterpolation(SGLeafNode* sourceNode, SGLeafNode* targetNode);

    std::shared_ptr<MeshInterpolatorFEM> mInterpolator;
    std::shared_ptr<MeshInterpolatorRenderModel> mInterpolatorModel;

    ApplicationControl* mAc;
};

#endif // INTERPOLATIONFEMDEMO_H
