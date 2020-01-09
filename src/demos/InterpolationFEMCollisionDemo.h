#ifndef INTERPOLATIONFEMCOLLISIONDEMO_H
#define INTERPOLATIONFEMCOLLISIONDEMO_H

#include <modules/demo_loader/Demo.h>
#include <scene/scene_graph/SGControl.h>

class ApplicationControl;
class MeshInterpolatorFEM;
class MeshInterpolatorRenderModel;


// Tests the interpolation collision feature:
// Two spheres are falling and rolling down multiple slopes while colliding
// with each other and the static environment. The sphere are simulated
// as deformable cubes (each consisting of 5 tetrahedrons). For the collision
// geometry the spheres are used. For the interpolation from cube to sphere
// a MeshInterpolatorFEM is used.
class InterpolationFEMCollisionDemo : public Demo
{
public:
    InterpolationFEMCollisionDemo(ApplicationControl& ac);

    // Demo interface
public:
    virtual std::string getName();
    virtual void load();
    virtual void unload();

private:

    std::shared_ptr<MeshInterpolatorFEM> addInterpolation(
            SGLeafNode* sourceNode, SGLeafNode* targetNode);

    ApplicationControl& mAc;
};

#endif // INTERPOLATIONFEMCOLLISIONDEMO_H
