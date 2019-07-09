#include "GeometryInfoUIControl.h"
#include "GeometryInfoUIForm.h"

#include <ui/selection/SelectionControl.h>

GeometryInfoUIControl::GeometryInfoUIControl(SelectionControl* sc)
    : mSc(sc)
{

}

void GeometryInfoUIControl::init(QWidget* widget)
{
    mForm = new GeometryInfoUIForm(widget);
    mSc->addListener(this);
}

void GeometryInfoUIControl::finalize()
{
    mSc->removeListener(this);
}

QWidget* GeometryInfoUIControl::getWidget()
{
    return mForm;
}

void GeometryInfoUIControl::onSceneNodeSelected(
        const std::shared_ptr<SceneData>& /*sd*/)
{
    // We are not interested in scene nodes for now.
}

void GeometryInfoUIControl::onSelectedSceneNodesChanged(
        const std::set<std::shared_ptr<SceneData> >& /*sd*/)
{
    // We are not interested in scene nodes for now.
}

void GeometryInfoUIControl::onSelectedVerticesChanged(
        const std::map<std::shared_ptr<SceneLeafData>, std::vector<ID> >& sv)
{
    mForm->onSelectedVerticesChanged(sv);
}
