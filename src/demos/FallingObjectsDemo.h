#ifndef FALLINGOBJECTSDEMO_H
#define FALLINGOBJECTSDEMO_H

#include <modules/demo_loader/Demo.h>

#include <data_structures/BoundingBox.h>

#include <scene/scene_graph/SGCore.h>

class ApplicationControl;

class FallingObjectsDemo : public Demo
{
public:
    FallingObjectsDemo(ApplicationControl* ac, std::string name, bool rigid);

    // Demo interface
public:
    virtual std::string getName();
    virtual void load();
    virtual void unload();

private:
    ApplicationControl* mAc;

    std::string mName;

    bool mRigid;

    SGLeafNode* importAndScale(
            SGChildrenNode* parent,
            std::string path,
            const std::array<float, 4>& color,
            double targetWidth,
            Eigen::Affine3d transform,
            bool rigid);
};

#endif // FALLINGOBJECTSDEMO_H
