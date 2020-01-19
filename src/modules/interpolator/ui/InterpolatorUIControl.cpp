#include "InterpolatorUIControl.h"
#include "InterpolatorUIForm.h"

#include <ui/selection/SelectionControl.h>

#include <modules/interpolator/InterpolatorModule.h>

InterpolatorUIControl::InterpolatorUIControl(
        InterpolatorModule* interpolatorModule)
    : mInterpolatorModule(interpolatorModule)
{

}

void InterpolatorUIControl::init(
        QWidget* widget, SelectionControl* selectionControl)
{
    mForm = new InterpolatorUIForm(widget);
    mForm->init(this, selectionControl);
    mSelectionControl = selectionControl;
}

void InterpolatorUIControl::addInterpolator(SGNode* source, SGNode* target)
{
    mForm->addInterpolator(source, target);
}

void InterpolatorUIControl::removeInterpolator(SGNode* source, SGNode* target)
{
    mForm->removeInterpolator(source, target);
}

void InterpolatorUIControl::updateNodeName(
        SGNode* node, const std::string& name)
{
    mForm->updateNodeName(node, name);
}

void InterpolatorUIControl::onUiAddInterpolationAction(
        SGNode* source, SGNode* target, MeshInterpolator::Type type)
{
    mInterpolatorModule->addInterpolator(source, target, type);
}

void InterpolatorUIControl::onUiRemoveInterpolationAction(
        SGNode* source, SGNode* target)
{
    mInterpolatorModule->removeInterpolator(source, target);
}

void InterpolatorUIControl::onSelectNodeAction(SGNode* node)
{
    mSelectionControl->selectSceneNode(node);
}

void InterpolatorUIControl::onSelectNodesAction(const std::vector<SGNode*>& nodes)
{
    mSelectionControl->selectSceneNodes(nodes);
}

void InterpolatorUIControl::finalize()
{
}

QWidget* InterpolatorUIControl::getWidget()
{
    return mForm;
}
