#ifndef INTERPOLATORMODULE_H
#define INTERPOLATORMODULE_H

#include <modules/Module.h>
#include <scene/data/geometric/MeshInterpolator.h>
#include <scene/scene_graph/SGCore.h>

#include <map>
#include <memory>

class InterpolatorUIControl;
class AbstractPolygon;

class InterpolatorModule : public Module, SGNodeListener
{
public:
    InterpolatorModule();

    // Adds a mesh interpolator between the geometries of source and target nodes
    // and returns it. If the sources and targets geometries don't align with
    // the MeshInterpolation type, the operation will be unsuccessful.
    // If successful, the UI is updates accordingly, else nothing happens and
    // returns nullptr.
    std::shared_ptr<MeshInterpolator> addInterpolator(
            SGNode* source,
            SGNode* target,
            const MeshInterpolator::Type& type);

    // Removes the mesh interpolator that interpolates source to target.
    // Updates the UI and removes the interpolator from the simulation.
    void removeInterpolator(SGNode* source, SGNode* target);

    void removeInterpolator(SGNode* node);

    // Removes all mesh interpolators.
    // Updates the UI and removes the interpolators from the simulation.
    void clearInterpolators();

    // Set the interpolator that references the given polygon as target visible.
    void setInterpolatorVisible(const std::shared_ptr<AbstractPolygon>& poly, bool visible);

    // Module interface
public:
    virtual void init(ApplicationControl* ac);
    virtual void initUI(QWidget* parent);
    virtual void finalize();
    virtual std::string getName();
    virtual QWidget* getWidget();

    // SGNodeListener interface
public:
    virtual void notifyParentChanged(SGNode* source, SGNode* parent);
    virtual void notifyNameChanged(SGNode* source, std::string name);
    virtual void notifyTreeChanged(SGNode* source, SGTree* tree);

private:
    ApplicationControl* mAc;
    std::shared_ptr<InterpolatorUIControl> mUIControl;

    std::map<std::pair<SGNode*, SGNode*>, std::shared_ptr<MeshInterpolator>> mInterpolatorMap;
};

#endif // INTERPOLATORMODULE_H
