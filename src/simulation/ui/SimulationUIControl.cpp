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
#include <scene/model/PolygonRenderModel.h>
#include <scene/model/RenderModel.h>
#include <scene/model/RenderModelVisitor.h>
#include <ui/qt/AbstractQtMemberWidget.h>
#include <ui/qt/QtMembersWidget.h>
#include <ui/qt/QtOwnersMembersWidget.h>
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
    mAc->getUIControl()->getSelectionControl()->addListener(this);

    mWidget->getMembersWidget()->addBool(
                "Simulation Paused",
                MemberAccessorFactory::createGetterSetter<bool, SimulationControl>(
                    &SimulationControl::isSimulationPaused,
                    &SimulationControl::setSimulationPaused,
                    true,
                    mAc->getSimulationControl(),
                    MemberAccessorFactory::createBoolComparator(),
                    mAc->getSimulationControl()->getDomain()));

    mWidget->getMembersWidget()->addDouble(
                "Time Step Size",
                MemberAccessorFactory::createGetterSetter<double, SimulationControl>(
                    &SimulationControl::getStepSize,
                    &SimulationControl::setStepSize,
                    0.01,
                    mAc->getSimulationControl(),
                    MemberAccessorFactory::createDoubleComparator(),
                    mAc->getSimulationControl()->getDomain()),
                1e-5, 10.0, 0.01, 4);

    mWidget->getMembersWidget()->addDouble(
                "Max. Constraint Error",
                MemberAccessorFactory::createGetterSetter<double, SimulationControl>(
                    &SimulationControl::getMaxConstraintError,
                    &SimulationControl::setMaxConstraintError,
                    0.0,
                    mAc->getSimulationControl(),
                    MemberAccessorFactory::createDoubleComparator(),
                    mAc->getSimulationControl()->getDomain()),
                0.0, 1.0, 1e-5, 7);

    mWidget->getMembersWidget()->addInteger(
                "Max. Constraint Iterations",
                MemberAccessorFactory::createGetterSetter<int, SimulationControl>(
                    &SimulationControl::getMaxNumConstraintSolverIterations,
                    &SimulationControl::setMaxNumConstraintSolverIterations,
                    1,
                    mAc->getSimulationControl(),
                    MemberAccessorFactory::createIntComparator(),
                    mAc->getSimulationControl()->getDomain()),
                0, 100, 1);

    mWidget->getMembersWidget()->addInteger(
                "FEM Correction Iterations",
                MemberAccessorFactory::createGetterSetter<int, SimulationControl>(
                    &SimulationControl::getNumFEMCorrectionIterations,
                    &SimulationControl::setNumFEMCorrectionIterations,
                    1,
                    mAc->getSimulationControl(),
                    MemberAccessorFactory::createIntComparator(),
                    mAc->getSimulationControl()->getDomain()),
                0, 100, 1);

    mWidget->getMembersWidget()->addVectorDouble(
                "Gravity",
                MemberAccessorFactory::createGetterSetter<Eigen::Vector3d, SimulationControl>(
                    &SimulationControl::getGravity,
                    &SimulationControl::setGravity,
                    Eigen::Vector3d::Zero(),
                    mAc->getSimulationControl(),
                    MemberAccessorFactory::createVectorDoubleComparator(),
                    mAc->getSimulationControl()->getDomain()),
                -100.0, 100.0, 1.0);

    mWidget->getMembersWidget()->addBool(
                "Invert normals if necessary",
                MemberAccessorFactory::createGetterSetter<bool, SimulationControl>(
                    &SimulationControl::getInvertNormalsIfNecessary,
                    &SimulationControl::setInvertNormalsIfNecessary,
                    true,
                    mAc->getSimulationControl(),
                    MemberAccessorFactory::createBoolComparator(),
                    mAc->getSimulationControl()->getDomain()));

    mWidget->getMembersWidget()->addBool(
                "Visualize Face normals",
                MemberAccessorFactory::createGetterSetter<bool, SGUIControl>(
                    &SGUIControl::isVisualizeFaceNormals,
                    &SGUIControl::setVisualizeFaceNormals,
                    false,
                    mAc->getSGUIControl(),
                    MemberAccessorFactory::createBoolComparator(),
                    mAc->getSimulationControl()->getDomain()));

    mWidget->getMembersWidget()->addBool(
                "Visualize Vertex normals",
                MemberAccessorFactory::createGetterSetter<bool, SGUIControl>(
                    &SGUIControl::isVisualizeVertexNormals,
                    &SGUIControl::setVisualizeVertexNormals,
                    false,
                    mAc->getSGUIControl(),
                    MemberAccessorFactory::createBoolComparator(),
                    mAc->getSimulationControl()->getDomain()));

    QtOwnersMembersWidget* omw = mWidget->getOwnersMembersWidget();
    // SimulationObjects
    QtMembersWidget* femWidget = omw->registerMembersWidget("FEMObject");
    QtMembersWidget* rigidBodyWidget = omw->registerMembersWidget("RigidBody");
    // RenderModels
    QtMembersWidget* polyModelWidget = omw->registerMembersWidget("PolygonRenderModel");

    femWidget->addDouble(
                "Youngs Modulus",
                MemberAccessorFactory::createGetterSetter<double, FEMObject>(
                    &FEMObject::getYoungsModulus,
                    &FEMObject::setYoungsModulus,
                    1e-3,
                    nullptr,
                    MemberAccessorFactory::createDoubleComparator(),
                    mAc->getSimulationControl()->getDomain()),
                0.0, 1e+5, 100.0, 3);

    femWidget->addDouble(
                "Poisson Ratio",
                MemberAccessorFactory::createGetterSetter<double, FEMObject>(
                    &FEMObject::getPoissonRatio,
                    &FEMObject::setPoissonRatio,
                    1e-3,
                    nullptr,
                    MemberAccessorFactory::createDoubleComparator(),
                    mAc->getSimulationControl()->getDomain()),
                0.0, 0.499, 0.05, 4);

    rigidBodyWidget->addBool(
                "Static",
                MemberAccessorFactory::createGetterSetter<bool, RigidBody>(
                    &RigidBody::isStatic,
                    &RigidBody::setStatic,
                    false,
                    nullptr,
                    nullptr,
                    mAc->getSimulationControl()->getDomain()));

    rigidBodyWidget->addDouble(
                "Mass",
                MemberAccessorFactory::createGetterSetter<double, RigidBody>(
                    &RigidBody::getMass,
                    &RigidBody::setMass,
                    1e-3,
                    nullptr,
                    MemberAccessorFactory::createDoubleComparator(),
                    mAc->getSimulationControl()->getDomain()),
                1e-5, 1e+5, 0.1, 5);

    // No domain needed. The render models already thread safety.
    polyModelWidget->addBool(
                "Render Face Normals",
                MemberAccessorFactory::createGetterSetter<bool, PolygonRenderModel>(
                    &PolygonRenderModel::isRenderFaceNormals,
                    &PolygonRenderModel::setRenderFaceNormals,
                    false, nullptr, nullptr, nullptr));

    polyModelWidget->addBool(
                "Render Vertex Normals",
                MemberAccessorFactory::createGetterSetter<bool, PolygonRenderModel>(
                    &PolygonRenderModel::isRenderVertexNormals,
                    &PolygonRenderModel::setRenderVertexNormals,
                    false, nullptr, nullptr, nullptr));

    polyModelWidget->addBool(
                "Only Outer Triangles",
                MemberAccessorFactory::createGetterSetter<bool, PolygonRenderModel>(
                    &PolygonRenderModel::isRenderOnlyOuterFaces,
                    &PolygonRenderModel::setRenderOnlyOuterFaces,
                    false, nullptr, nullptr, nullptr));

    omw->revalidate();

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

void SimulationUIControl::onSceneNodeSelected(
        const std::shared_ptr<SceneData>& sd)
{
    std::vector<std::shared_ptr<SceneData>> sdsVec;
    sdsVec.push_back(sd);
    onSelectedSceneNodesChanged(sdsVec);
}

void SimulationUIControl::onSelectedSceneNodesChanged(
        const std::set<std::shared_ptr<SceneData> >& sds)
{
    std::vector<std::shared_ptr<SceneData>> sdsVec;
    sdsVec.reserve(sds.size());
    for (const std::shared_ptr<SceneData>& sd : sds)
    {
        sdsVec.push_back(sd);
    }
    onSelectedSceneNodesChanged(sdsVec);
}

void SimulationUIControl::onSelectedVerticesChanged(
        const std::map<std::shared_ptr<SceneLeafData>, std::vector<ID> >& /*sv*/)
{
    // Nothing to do here.
}

void SimulationUIControl::onSelectedSceneNodesChanged(
        const std::vector<std::shared_ptr<SceneData> >& sds)
{
    QtOwnersMembersWidget* omw = mWidget->getOwnersMembersWidget();
    QtMembersWidget* femWidget = omw->getMembersWidget("FEMObject");
    QtMembersWidget* rigidWidget = omw->getMembersWidget("RigidBody");
    QtMembersWidget* polyModelWidget = omw->getMembersWidget("PolygonRenderModel");

    femWidget->clearOwners();
    rigidWidget->clearOwners();
    polyModelWidget->clearOwners();

    for (const std::shared_ptr<SceneData>& sd : sds)
    {
        if (sd->isLeafData())
        {
            std::shared_ptr<SceneLeafData> leafData =
                    std::static_pointer_cast<SceneLeafData>(sd);

            // RenderModels
            std::shared_ptr<RenderModel> rm = leafData->getRenderModel();
            if (rm)
            {
                class RMV : public RenderModelVisitor
                {
                public:
                    RMV(QtMembersWidget* _polyModel)
                        : polyModel(_polyModel)
                    {

                    }

                    void visit(PolygonRenderModel& model)
                    {
                        polyModel->addOwner(&model);
                        polyModel->updateValues();
                    }

                    void visit(LinearForceRenderModel& /*model*/)
                    {

                    }

                private:
                    QtMembersWidget* polyModel;
                } visitor(polyModelWidget);
                rm->accept(visitor);
            }

            // SimulationObjects
            std::shared_ptr<SimulationObject> so = leafData->getSimulationObject();
            if (so)
            {
                switch (so->getType())
                {
                case SimulationObject::Type::FEM_OBJECT:
                {
                    std::shared_ptr<FEMObject> femObj =
                            std::static_pointer_cast<FEMObject>(so);

                    femWidget->addOwner(femObj.get());
                    femWidget->updateValues();
                    break;
                }
                case SimulationObject::Type::RIGID_BODY:
                {
                    std::shared_ptr<RigidBody> rigid =
                            std::static_pointer_cast<RigidBody>(so);
                    rigidWidget->addOwner(rigid.get());
                    rigidWidget->updateValues();
                    break;
                }
                case SimulationObject::Type::SIMULATION_POINT:
                {
                    break;
                }
                }

            }
        }
    }
}
