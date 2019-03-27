#include "SimulationUIControl.h"
#include "SimulationUIWidget.h"

#include <ApplicationControl.h>

#include <QButtonGroup>
#include <iostream>

#include <ui/UIControl.h>

#include <ui/selection/SelectionControl.h>
#include <ui/selection/SelectionVertices.h>

#include <scene/scene_graph/SGCore.h>
#include <scene/scene_graph/SceneData.h>
#include <scene/VertexCollection.h>
#include <scene/data/GeometricData.h>
#include <scene/data/GeometricDataVisitor.h>
#include <scene/data/geometric/Polygon3D.h>
#include <simulation/SimulationObjectFactory.h>
#include <simulation/rigid/RigidBody.h>
#include <scene/model/RenderModel.h>

#include <simulation/fem/FEMObject.h>


SimulationUIControl::SimulationUIControl(
        SimulationModule* module,
        ApplicationControl* ac)
    : mModule(module)
    , mAc(ac)
{

}

void SimulationUIControl::init(QWidget* parent)
{
    mWidget = new SimulationUIWidget(this, parent);
    mAc->getSimulationControl()->addListener(this);
}

QWidget* SimulationUIControl::getWidget()
{
    return mWidget;
}

void SimulationUIControl::onSimulationButtonToggled(bool checked)
{
    mAc->getSimulationControl()->setSimulationPaused(!checked);
}

void SimulationUIControl::onTruncateButtonClicked()
{
    // For now: add truncation
    mAc->getSimulationControl()->clearTruncations();
    const std::map<SceneLeafData*, std::vector<ID>>& dvm =
            mAc->getUIControl()->getSelectionControl()->getSelectionVertices()
            ->getSelectedVertexCollection()->getDataVectorsMap();
    for (std::map<SceneLeafData*, std::vector<ID>>::const_iterator it = dvm.begin();
         it != dvm.end(); ++it)
    {
        mAc->getSimulationControl()->addTruncations(
                    dynamic_cast<FEMObject*>(
                        it->first->getSimulationObjectRaw()), it->second);
    }
}

void SimulationUIControl::onCreateFEMObjectClicked()
{
    SelectionControl* sc = mAc->getUIControl()->getSelectionControl();

    for (SceneData* sd : sc->getSelectedSceneData())
    {
        if (sd->isLeafData())
        {
            SceneLeafData* leafData = static_cast<SceneLeafData*>(sd);
            mAc->getSGControl()->createFEMObject(leafData);
        }
        else
        {
            std::cout << "Selected node has no geometry." << std::endl;
        }
    }
}

void SimulationUIControl::onCreateRigidObjectClicked(double mass)
{
    SelectionControl* sc = mAc->getUIControl()->getSelectionControl();

    for (SceneData* sd : sc->getSelectedSceneData())
    {
        if (sd->isLeafData())
        {
            SceneLeafData* leafData = static_cast<SceneLeafData*>(sd);
            mAc->getSGControl()->createRigidBody(leafData, mass);
        }
        else
        {
            std::cout << "Selected node has no geometry." << std::endl;
        }
    }
}

void SimulationUIControl::onRemoveSimulationObjectClicked()
{
    SimulationObject* so = mWidget->getSelectedSimulationObject();
    if (so)
    {
        mAc->getSimulationControl()->removeSimulationObject(so->shared_from_this());
    }
    else
    {
        std::cout << "Can not remove simulation object because it is already removed.\n";
    }
}

void SimulationUIControl::onCreateCollidableClicked()
{
    SelectionControl* sc = mAc->getUIControl()->getSelectionControl();

    for (SceneData* sd : sc->getSelectedSceneData())
    {
        if (sd->isLeafData())
        {
            SceneLeafData* leafData = static_cast<SceneLeafData*>(sd);
            mAc->getSGControl()->createCollidable(leafData);
        }
        else
        {
            std::cout << "Selected node has no geometry." << std::endl;
        }
    }
}

void SimulationUIControl::onChangeCollisionRenderingLevel(int level)
{
    mAc->getSimulationControl()->setCollisionRenderingLevel(level);
}

void SimulationUIControl::onEnableCollisionRendering(bool enable)
{
    mAc->getSimulationControl()->setBVHCollisionRenderingEnabled(enable);
}

void SimulationUIControl::onSimulationObjectAdded(SimulationObject* so)
{
    mWidget->onSimulationObjectAdded(so);
}

void SimulationUIControl::onSimulationObjectRemoved(SimulationObject* so)
{
    mWidget->onSimulationObjectRemoved(so);
}

void SimulationUIControl::onConstraintAdded(SimulationObject* /*so*/)
{
    // TODO: implement what happens in the UI when a constraintis added
}

void SimulationUIControl::onConstraintRemoved(SimulationObject* /*so*/)
{
    // TODO: implement what happens in the UI when a constraint is removed
}
