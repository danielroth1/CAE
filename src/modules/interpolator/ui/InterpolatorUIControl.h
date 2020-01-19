#ifndef INTERPOLATORUICONTROL_H
#define INTERPOLATORUICONTROL_H

#include <scene/data/geometric/MeshInterpolator.h>
#include <scene/scene_graph/SGCore.h>
#include <ui/selection/SelectionListener.h>

class InterpolatorModule;
class InterpolatorUIForm;
class Polygon;
class QWidget;
class SelectionControl;

// This class has two functions:
// - provide operations to update the UI
// - provide operations for the UI to react on UI events. These methods start
//      with "on" and should only be called by Qt classes.
class InterpolatorUIControl
{
public:
    InterpolatorUIControl(InterpolatorModule* interpolatorModule);

    // Adds itself as listener to SelectionControl.
    void init(QWidget* widget,
              SelectionControl* selectionControl);

    // Adds interpolator to UI
    void addInterpolator(SGNode* source, SGNode* target);

    // Remove all interpolations
    void removeInterpolator(SGNode* source, SGNode* target);

    // Updates interpolator entries of the given nodes name.
    void updateNodeName(SGNode* node, const std::string& name);

    // "Add interpolation" button clicked in the UI. Called by the widget.
    void onUiAddInterpolationAction(
            SGNode* source, SGNode* target, MeshInterpolator::Type type);

    // "Remove interpolation" button clicked in the UI. Called by the widget.
    void onUiRemoveInterpolationAction(SGNode* source, SGNode* target);

    // Selects the given node.
    void onSelectNodeAction(SGNode* node);
    void onSelectNodesAction(const std::vector<SGNode*>& nodes);

    void finalize();

    QWidget* getWidget();

private:
    InterpolatorModule* mInterpolatorModule;
    SelectionControl* mSelectionControl;
    InterpolatorUIForm* mForm;
};

#endif // INTERPOLATORUICONTROL_H
