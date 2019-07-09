#ifndef INTERPOLATIONMESHMESHDEMO_H
#define INTERPOLATIONMESHMESHDEMO_H

#include <modules/demo_loader/Demo.h>
#include <scene/scene_graph/SGControl.h>

class ApplicationControl;
class MeshInterpolatorMeshMesh;
class MeshInterpolatorRenderModel;


class InterpolationMeshMeshDemo : public Demo
{
public:
    InterpolationMeshMeshDemo(ApplicationControl* ac);

    // Demo interface
public:
    virtual std::string getName();
    virtual void load();
    virtual void unload();

private:

    void addInterpolation(SGLeafNode* sourceNode, SGLeafNode* targetNode);

    std::shared_ptr<MeshInterpolatorMeshMesh> mInterpolator;
    std::shared_ptr<MeshInterpolatorRenderModel> mInterpolatorModel;

    ApplicationControl* mAc;
};

#endif // INTERPOLATIONMESHMESHDEMO_H
