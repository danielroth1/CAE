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
#include <ui/qt/QtMembersWidget.h>
#include <utils/MemberAccessorFactory.h>
#include <simulation/collision_detection/CollisionManager.h>
#include <ui/scene_graph/SGUIControl.h>

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

    mWidget->getMembersWidget()->addBool(
                "Simulation Paused",
                MemberAccessorFactory::createGetterSetter<bool, SimulationControl>(
                    &SimulationControl::isSimulationPaused,
                    &SimulationControl::setSimulationPaused,
                    mAc->getSimulationControl(),
                    mAc->getSimulationControl()->getDomain()));

    mWidget->getMembersWidget()->addDouble(
                "Time Step Size",
                MemberAccessorFactory::createGetterSetter<double, SimulationControl>(
                    &SimulationControl::getStepSize,
                    &SimulationControl::setStepSize,
                    mAc->getSimulationControl(),
                    mAc->getSimulationControl()->getDomain()),
                1e-5, 10.0, 0.01, 4);

    mWidget->getMembersWidget()->addDouble(
                "Max. Constraint Error",
                MemberAccessorFactory::createGetterSetter<double, SimulationControl>(
                    &SimulationControl::getMaxConstraintError,
                    &SimulationControl::setMaxConstraintError,
                    mAc->getSimulationControl(),
                    mAc->getSimulationControl()->getDomain()),
                0.0, 1.0, 1e-5, 7);

    mWidget->getMembersWidget()->addInteger(
                "Max. Constraint Iterations",
                MemberAccessorFactory::createGetterSetter<int, SimulationControl>(
                    &SimulationControl::getMaxNumConstraintSolverIterations,
                    &SimulationControl::setMaxNumConstraintSolverIterations,
                    mAc->getSimulationControl(),
                    mAc->getSimulationControl()->getDomain()),
                0, 100, 1);

    mWidget->getMembersWidget()->addInteger(
                "FEM Correction Iterations",
                MemberAccessorFactory::createGetterSetter<int, SimulationControl>(
                    &SimulationControl::getNumFEMCorrectionIterations,
                    &SimulationControl::setNumFEMCorrectionIterations,
                    mAc->getSimulationControl(),
                    mAc->getSimulationControl()->getDomain()),
                0, 100, 1);

    mWidget->getMembersWidget()->addVectorDouble(
                "Gravity",
                MemberAccessorFactory::createGetterSetter<Eigen::Vector3d, SimulationControl>(
                    &SimulationControl::getGravity,
                    &SimulationControl::setGravity,
                    mAc->getSimulationControl(),
                    mAc->getSimulationControl()->getDomain()),
                -100.0, 100.0, 1.0);

    mWidget->getMembersWidget()->addBool(
                "Invert normals if necessary",
                MemberAccessorFactory::createGetterSetter<bool, SimulationControl>(
                    &SimulationControl::getInvertNormalsIfNecessary,
                    &SimulationControl::setInvertNormalsIfNecessary,
                    mAc->getSimulationControl(),
                    mAc->getSimulationControl()->getDomain()));

    mWidget->getMembersWidget()->addBool(
                "Visualize Face normals",
                MemberAccessorFactory::createGetterSetter<bool, SGUIControl>(
                    &SGUIControl::isVisualizeFaceNormals,
                    &SGUIControl::setVisualizeFaceNormals,
                    mAc->getSGUIControl(),
                    mAc->getSimulationControl()->getDomain()));

    mWidget->getMembersWidget()->addBool(
                "Visualize Vertex normals",
                MemberAccessorFactory::createGetterSetter<bool, SGUIControl>(
                    &SGUIControl::isVisualizeVertexNormals,
                    &SGUIControl::setVisualizeVertexNormals,
                    mAc->getSGUIControl(),
                    mAc->getSimulationControl()->getDomain()));
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
    const std::map<std::shared_ptr<SceneLeafData>, std::vector<ID>>& dvm =
            mAc->getUIControl()->getSelectionControl()->getSelectionVertices()
            ->getSelectedVertexCollection()->getDataVectorsMap();
    for (std::map<std::shared_ptr<SceneLeafData>, std::vector<ID>>::const_iterator it = dvm.begin();
         it != dvm.end(); ++it)
    {
        if (!it->first->getSimulationObjectRaw() ||
            it->first->getSimulationObjectRaw()->getType() != SimulationObject::Type::FEM_OBJECT)
            continue;

        mAc->getSimulationControl()->addTruncations(
                    dynamic_cast<FEMObject*>(
                        it->first->getSimulationObjectRaw()), it->second);
    }
}

void SimulationUIControl::onCreateFEMObjectClicked()
{
    SelectionControl* sc = mAc->getUIControl()->getSelectionControl();

    for (const std::shared_ptr<SceneData>& sd : sc->getSelectedSceneData())
    {
        if (sd->isLeafData())
        {
            std::shared_ptr<SceneLeafData> leafData =
                    std::static_pointer_cast<SceneLeafData>(sd);
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

    for (const std::shared_ptr<SceneData>& sd : sc->getSelectedSceneData())
    {
        if (sd->isLeafData())
        {
            std::shared_ptr<SceneLeafData> leafData =
                    std::static_pointer_cast<SceneLeafData>(sd);
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

    for (const std::shared_ptr<SceneData>& sd : sc->getSelectedSceneData())
    {
        if (sd->isLeafData())
        {
            std::shared_ptr<SceneLeafData> leafData =
                    std::static_pointer_cast<SceneLeafData>(sd);
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
