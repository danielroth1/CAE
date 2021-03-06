#include "SimulationUIControl.h"
#include "SimulationUIWidget.h"

#include <ApplicationControl.h>

#include <QButtonGroup>
#include <SimulationControl.h>
#include <iostream>

#include <ui/UIControl.h>

#include <ui/selection/SelectionControl.h>
#include <ui/selection/SelectionVertices.h>

#include <scene/scene_graph/SGCore.h>
#include <scene/scene_graph/SceneData.h>
#include <scene/data/GeometricData.h>
#include <scene/data/GeometricDataVisitor.h>
#include <scene/data/geometric/MeshInterpolationManager.h>
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
#include <simulation/constraints/Constraint.h>

#include <simulation/fem/FEMObject.h>
#include <simulation/fem/FEMSimulation.h>

#include <memory>


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
                0.0, 1.0, 1e-5, 10);

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
                "FEM Collision Iterations",
                MemberAccessorFactory::createGetterSetter<int, SimulationControl>(
                    &SimulationControl::getNumFEMCorrectionIterations,
                    &SimulationControl::setNumFEMCorrectionIterations,
                    1,
                    mAc->getSimulationControl(),
                    MemberAccessorFactory::createIntComparator(),
                    mAc->getSimulationControl()->getDomain()),
                0, 100, 1);

    mWidget->getMembersWidget()->addDouble(
                "Col. Correction Factor",
                MemberAccessorFactory::createGetterSetter<double, SimulationControl>(
                    &SimulationControl::getPositionCorrectionFactor,
                    &SimulationControl::setPositionCorrectionFactor,
                    0.2,
                    mAc->getSimulationControl(),
                    MemberAccessorFactory::createDoubleComparator(),
                    mAc->getSimulationControl()->getDomain()),
                0.0, 1.0, 1e-1, 3);

    mWidget->getMembersWidget()->addDouble(
                "Col. Margin",
                MemberAccessorFactory::createGetterSetter<double, SimulationControl>(
                    &SimulationControl::getCollisionMargin,
                    &SimulationControl::setCollisionMargin,
                    1e-2,
                    mAc->getSimulationControl(),
                    MemberAccessorFactory::createDoubleComparator(),
                    mAc->getSimulationControl()->getDomain()),
                0.0, 10.0, 1e-2, 5);

    mWidget->getMembersWidget()->addDouble(
                "Contact Margin",
                MemberAccessorFactory::createGetterSetter<double, SimulationControl>(
                    &SimulationControl::getContactMargin,
                    &SimulationControl::setContactMargin,
                    1e-3,
                    mAc->getSimulationControl(),
                    MemberAccessorFactory::createDoubleComparator(),
                    mAc->getSimulationControl()->getDomain()),
                0.0, 10.0, 1e-3, 5);

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
                "Warm Starting",
                MemberAccessorFactory::createGetterSetter<bool, SimulationControl>(
                    &SimulationControl::isWarmStaring,
                    &SimulationControl::setWarmStarting,
                    true,
                    mAc->getSimulationControl(),
                    MemberAccessorFactory::createBoolComparator(),
                    mAc->getSimulationControl()->getDomain()));

    mWidget->getMembersWidget()->addBool(
                "Invert Normals if necessary",
                MemberAccessorFactory::createGetterSetter<bool, SimulationControl>(
                    &SimulationControl::getInvertNormalsIfNecessary,
                    &SimulationControl::setInvertNormalsIfNecessary,
                    true,
                    mAc->getSimulationControl(),
                    MemberAccessorFactory::createBoolComparator(),
                    mAc->getSimulationControl()->getDomain()));

    mWidget->getMembersWidget()->addBool(
                "Visualize Face Normals",
                MemberAccessorFactory::createGetterSetter<bool, SGUIControl>(
                    &SGUIControl::isVisualizeFaceNormals,
                    &SGUIControl::setVisualizeFaceNormals,
                    false,
                    mAc->getSGUIControl(),
                    MemberAccessorFactory::createBoolComparator(),
                    mAc->getSimulationControl()->getDomain()));

    mWidget->getMembersWidget()->addBool(
                "Visualize Vertex Normals",
                MemberAccessorFactory::createGetterSetter<bool, SGUIControl>(
                    &SGUIControl::isVisualizeVertexNormals,
                    &SGUIControl::setVisualizeVertexNormals,
                    false,
                    mAc->getSGUIControl(),
                    MemberAccessorFactory::createBoolComparator(),
                    mAc->getSimulationControl()->getDomain()));

    mWidget->getMembersWidget()->addBool(
                "Visualize Collision Normals",
                MemberAccessorFactory::createGetterSetter<bool, SimulationControl>(
                    &SimulationControl::isCollisionNormalsVisible,
                    &SimulationControl::setCollisionNormalsVisible,
                    true,
                    mAc->getSimulationControl(),
                    MemberAccessorFactory::createBoolComparator(),
                    mAc->getSimulationControl()->getDomain()));

    mWidget->getMembersWidget()->addBool(
                "Visualize Interpolators",
                MemberAccessorFactory::createGetterSetter<bool, MeshInterpolationManager>(
                    &MeshInterpolationManager::isInterpolatorsVisible,
                    &MeshInterpolationManager::setInterpolatorsVisible,
                    false,
                    mAc->getMeshInterpolationManager(),
                    MemberAccessorFactory::createBoolComparator(),
                    mAc->getSimulationControl()->getDomain()));

    QtOwnersMembersWidget* omw = mWidget->getOwnersMembersWidget();

    QtMembersWidget* sceneNodeWidget = omw->registerMembersWidget("SceneData");
    // SimulationObjects
    QtMembersWidget* simulationObjectWidget = omw->registerMembersWidget("SimulationObject");
    QtMembersWidget* femWidget = omw->registerMembersWidget("FEMObject");
    QtMembersWidget* rigidBodyWidget = omw->registerMembersWidget("RigidBody");
    // RenderModels
    QtMembersWidget* polyModelWidget = omw->registerMembersWidget("PolygonRenderModel");

    omw->setMembersWidgetVisible("SceneData", false);
    omw->setMembersWidgetVisible("FEMObject", false);
    omw->setMembersWidgetVisible("RigidBody", false);
    omw->setMembersWidgetVisible("PolygonRenderModel", false);

//    sceneNodeWidget->addBool(
//                "Selectable",
//                MemberAccessorFactory::createGetterSetter<bool, SceneData>(
//                    &SceneData::isSceneDataSelectable,
//                    &SceneData::setSceneDataSelectable,
//                    true, nullptr, nullptr, nullptr));

    sceneNodeWidget->addBool(
                "Vertices Selectable",
                MemberAccessorFactory::createGetterSetter<bool, SceneData>(
                    &SceneData::isVerticesSelectable,
                    &SceneData::setVerticesSelectable,
                    true, nullptr, nullptr, nullptr));

    // -> Move it to SGControl.
    std::function<bool(SceneData*)> isCollidableFunction =
            [=](SceneData* sd)
    {
        if (sd->isLeafData())
        {
            SceneLeafData* leafData = static_cast<SceneLeafData*>(sd);

            if (leafData->getGeometricData() &&
                    leafData->getGeometricData()->getType() == GeometricData::Type::POLYGON)
            {
                return mAc->getSimulationControl()->isCollidable(
                            std::static_pointer_cast<AbstractPolygon>(leafData->getGeometricData()));
            }
        }
        return false;
    };
    std::function<void(SceneData*, bool)> setCollidableFunction =
            [=](SceneData* sd, bool collidable)
    {
        if (sd->isLeafData())
        {
            SceneLeafData* leafData = static_cast<SceneLeafData*>(sd);

            if (leafData->getGeometricData() &&
                    leafData->getGeometricData()->getType() == GeometricData::Type::POLYGON)
            {
                mAc->getSimulationControl()->setCollidable(
                            std::static_pointer_cast<AbstractPolygon>(leafData->getGeometricData()), collidable);
                updateMemberWidgets(mSceneDatasInUI);
            }
        }
    };
    sceneNodeWidget->addBool(
                "Collidable",
                MemberAccessorFactory::createGetterSetter<bool, SceneData>(
                    isCollidableFunction,
                    setCollidableFunction,
                    true,
                    nullptr,
                    MemberAccessorFactory::createBoolComparator(),
                    mAc->getSimulationControl()->getDomain()));

    std::function<int(SceneData*)> getSimulationObjectType =
            [=](SceneData* sd)
    {
        if (sd->isLeafData())
        {
            SceneLeafData* leafData = static_cast<SceneLeafData*>(sd);
            SimulationObject* so = leafData->getSimulationObjectRaw();
            if (so != nullptr)
            {
                return so->getType();
            }
        }
        return SimulationObject::Type::NONE;
    };
    std::function<void(SceneData*, int)> setSimulationObjectType =
            [=](SceneData* sd, int type)
    {
        if (sd->isLeafData())
        {
            std::shared_ptr<SceneLeafData> leafData =
                    std::static_pointer_cast<SceneLeafData>(sd->shared_from_this());
            if (type == SimulationObject::Type::FEM_OBJECT)
            {
                mAc->getSGControl()->createFEMObject(leafData);
            }
            else if (type == SimulationObject::Type::RIGID_BODY)
            {
                mAc->getSGControl()->createRigidBody(leafData, 1.0);
            }
            else
            {
                mAc->getSGControl()->removeSimulationObject(leafData);
            }

            updateMemberWidgets(mSceneDatasInUI);
        }
    };
    std::vector<std::string> enumNames =
    {
        "FEMObject", "SimulationPoint", "RigidBody", "None"
    };
    sceneNodeWidget->addEnumComboBox(
                "Simulation Object Type",
                MemberAccessorFactory::createGetterSetter<int, SceneData>(
                    getSimulationObjectType,
                    setSimulationObjectType,
                    static_cast<int>(SimulationObject::Type::NONE),
                    nullptr,
                    MemberAccessorFactory::createIntComparator(),
                    mAc->getSimulationControl()->getDomain()),
                enumNames);

    simulationObjectWidget->addDouble(
                "Mass",
                MemberAccessorFactory::createGetterSetter<double, SimulationObject>(
                    &SimulationObject::getMass,
                    &SimulationObject::setMass,
                    1e-3,
                    nullptr,
                    MemberAccessorFactory::createDoubleComparator(),
                    mAc->getSimulationControl()->getDomain()),
                1e-5, 1e+5, 0.1, 5);

    femWidget->addDouble(
                "Youngs Modulus",
                MemberAccessorFactory::createGetterSetter<double, FEMObject>(
                    &FEMObject::getYoungsModulus,
                    &FEMObject::setYoungsModulus,
                    1e-3,
                    nullptr,
                    MemberAccessorFactory::createDoubleComparator(),
                    mAc->getSimulationControl()->getDomain()),
                0.0, 1e+8, 100.0, 3);

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

    femWidget->addDouble(
                "Plastic Yield",
                MemberAccessorFactory::createGetterSetter<double, FEMObject>(
                    &FEMObject::getPlasticYield,
                    &FEMObject::setPlasticYield,
                    1e-3,
                    nullptr,
                    MemberAccessorFactory::createDoubleComparator(),
                    mAc->getSimulationControl()->getDomain()),
                0.0, 1e+5, 100.0, 3);

    femWidget->addDouble(
                "Plastic Creep",
                MemberAccessorFactory::createGetterSetter<double, FEMObject>(
                    &FEMObject::getPlasticCreep,
                    &FEMObject::setPlasticCreep,
                    1e-3,
                    nullptr,
                    MemberAccessorFactory::createDoubleComparator(),
                    mAc->getSimulationControl()->getDomain()),
                0.0, 1e+5, 100.0, 3);

    femWidget->addDouble(
                "Plastic Max. Strain",
                MemberAccessorFactory::createGetterSetter<double, FEMObject>(
                    &FEMObject::getPlasticMaxStrain,
                    &FEMObject::setPlasticMaxStrain,
                    1e-3,
                    nullptr,
                    MemberAccessorFactory::createDoubleComparator(),
                    mAc->getSimulationControl()->getDomain()),
                0.0, 1e+5, 100.0, 3);

    femWidget->addDouble(
                "Friction Static",
                MemberAccessorFactory::createGetterSetter<double, FEMObject>(
                    &FEMObject::getFrictionStatic,
                    &FEMObject::setFrictionStatic,
                    1e-3,
                    nullptr,
                    MemberAccessorFactory::createDoubleComparator(),
                    mAc->getSimulationControl()->getDomain()),
                0, 1e+5, 0.1, 5);

    femWidget->addDouble(
                "Friction Dynamic",
                MemberAccessorFactory::createGetterSetter<double, FEMObject>(
                    &FEMObject::getFrictionDynamic,
                    &FEMObject::setFrictionDynamic,
                    1e-3,
                    nullptr,
                    MemberAccessorFactory::createDoubleComparator(),
                    mAc->getSimulationControl()->getDomain()),
                0, 1e+5, 0.1, 5);

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
                "Friction Static",
                MemberAccessorFactory::createGetterSetter<double, RigidBody>(
                    &RigidBody::getFrictionStatic,
                    &RigidBody::setFrictionStatic,
                    1e-3,
                    nullptr,
                    MemberAccessorFactory::createDoubleComparator(),
                    mAc->getSimulationControl()->getDomain()),
                0, 1e+5, 0.1, 5);

    rigidBodyWidget->addDouble(
                "Friction Dynamic",
                MemberAccessorFactory::createGetterSetter<double, RigidBody>(
                    &RigidBody::getFrictionDynamic,
                    &RigidBody::setFrictionDynamic,
                    1e-3,
                    nullptr,
                    MemberAccessorFactory::createDoubleComparator(),
                    mAc->getSimulationControl()->getDomain()),
                0, 1e+5, 0.1, 5);

    // No domain needed. The render models already thread safety.
    polyModelWidget->addBool(
                "Wireframe",
                MemberAccessorFactory::createGetterSetter<bool, PolygonRenderModel>(
                    &PolygonRenderModel::isWireframeEnabled,
                    &PolygonRenderModel::setWireframeEnabled,
                    false, nullptr, nullptr, nullptr));

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

void SimulationUIControl::onSingleStepButtonClicked()
{
    mAc->getSimulationControl()->performSingleStep();
}

void SimulationUIControl::onTruncateButtonClicked()
{
    // For now: add truncation
    mAc->getSimulationControl()->clearTruncations();
    const std::map<std::shared_ptr<SceneLeafData>, std::vector<ID>>& dvm =
            mAc->getUIControl()->getSelectionControl()->getSelectionVertices()
            ->getDataVectorsMap();
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
        mAc->getSGControl()->removeSimulationObject(so->shared_from_this());
    }
    else
    {
        std::cout << "Cannot remove simulation object because there is none.\n";
    }
}

void SimulationUIControl::onRemoveConstraintClicked()
{
    Constraint* c = mWidget->getSelectedConstraint();
    if (c)
    {
        mAc->getSimulationControl()->removeConstraint(
                    std::static_pointer_cast<Constraint>(c->shared_from_this()));
    }
    else
    {
        std::cout << "Can not remove constraint because it is already removed.\n";
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

void SimulationUIControl::onPrintStiffnessMatrixClicked()
{
    for (const std::shared_ptr<SceneData>& sd :
         mAc->getUIControl()->getSelectionControl()->getSelectedSceneData())
    {
        if (sd->isLeafData())
        {
            std::shared_ptr<SceneLeafData> leafData =
                    std::static_pointer_cast<SceneLeafData>(sd);
            std::shared_ptr<SimulationObject> so = leafData->getSimulationObject();
            if (so->getType() == SimulationObject::FEM_OBJECT)
            {
                std::shared_ptr<FEMObject> femObj =
                        std::static_pointer_cast<FEMObject>(so);

                mAc->getSimulationControl()->getFEMSimulation()->
                        printStiffnessMatrix(femObj.get());
                // 40 char seperator
                std::cout << "========================================\n";
            }
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

void SimulationUIControl::onConstraintAdded(const std::shared_ptr<Constraint>& c)
{
    mWidget->onConstraintAdded(c);
}

void SimulationUIControl::onConstraintRemoved(const std::shared_ptr<Constraint>& c)
{
    mWidget->onConstraintRemoved(c);
}

void SimulationUIControl::onSceneNodeSelected(
        const std::shared_ptr<SceneData>& /*sd*/)
{
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

    updateMemberWidgets(sdsVec);
}

void SimulationUIControl::onSelectedVerticesChanged(
        const std::map<std::shared_ptr<SceneLeafData>, std::vector<ID> >& /*sv*/)
{
    // Nothing to do here.
}

void SimulationUIControl::onSelectedSceneNodesChanged(
        const std::vector<std::shared_ptr<SceneData> >& sds)
{
    updateMemberWidgets(sds);
}

void SimulationUIControl::updateMemberWidgets(
        const std::vector<std::shared_ptr<SceneData>>& sceneDatas)
{
    mSceneDatasInUI = sceneDatas;

    // Go over all selected scene nodes and inform the QtMemberWidgets that
    // update the ui elements over changed owners.

    QtOwnersMembersWidget* omw = mWidget->getOwnersMembersWidget();
    QtMembersWidget* sceneDataWidget = omw->getMembersWidget("SceneData");
    QtMembersWidget* simulationObjectWidget = omw->getMembersWidget("SimulationObject");
    QtMembersWidget* femWidget = omw->getMembersWidget("FEMObject");
    QtMembersWidget* rigidWidget = omw->getMembersWidget("RigidBody");
    QtMembersWidget* polyModelWidget = omw->getMembersWidget("PolygonRenderModel");

    sceneDataWidget->clearOwners();
    simulationObjectWidget->clearOwners();
    femWidget->clearOwners();
    rigidWidget->clearOwners();
    polyModelWidget->clearOwners();

    omw->setMembersWidgetVisible("SceneData", false);
    omw->setMembersWidgetVisible("SimulationObject", false);
    omw->setMembersWidgetVisible("FEMObject", false);
    omw->setMembersWidgetVisible("RigidBody", false);
    omw->setMembersWidgetVisible("PolygonRenderModel", false);

    for (auto it = sceneDatas.begin(); it != sceneDatas.end(); ++it)
    {
        const std::shared_ptr<SceneData>& sd = *it;
        sceneDataWidget->addOwner(sd.get());
        sceneDataWidget->updateValues();
        omw->setMembersWidgetVisible("SceneData", true);

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
                    RMV(QtMembersWidget* _polyModel,
                        QtOwnersMembersWidget* _omw)
                        : polyModel(_polyModel)
                        , omw(_omw)
                    {

                    }

                    void visit(PolygonRenderModel& model)
                    {
                        polyModel->addOwner(&model);
                        polyModel->updateValues();
                        omw->setMembersWidgetVisible("PolygonRenderModel", true);
                    }

                    void visit(LinearForceRenderModel& /*model*/)
                    {

                    }

                private:
                    QtMembersWidget* polyModel;
                    QtOwnersMembersWidget* omw;
                } visitor(polyModelWidget, omw);
                rm->accept(visitor);
            }

            // SimulationObjects
            std::shared_ptr<SimulationObject> so = leafData->getSimulationObject();
            if (so)
            {
                simulationObjectWidget->addOwner(so.get());
                simulationObjectWidget->updateValues();
                omw->setMembersWidgetVisible("SimulationObject", true);
                switch (so->getType())
                {
                case SimulationObject::Type::FEM_OBJECT:
                {
                    std::shared_ptr<FEMObject> femObj =
                            std::static_pointer_cast<FEMObject>(so);

                    femWidget->addOwner(femObj.get());
                    femWidget->updateValues();
                    omw->setMembersWidgetVisible("FEMObject", true);
                    break;
                }
                case SimulationObject::Type::RIGID_BODY:
                {
                    std::shared_ptr<RigidBody> rigid =
                            std::static_pointer_cast<RigidBody>(so);
                    rigidWidget->addOwner(rigid.get());
                    rigidWidget->updateValues();
                    omw->setMembersWidgetVisible("RigidBody", true);
                    break;
                }
                case SimulationObject::Type::SIMULATION_POINT:
                {
                    break;
                }
                case SimulationObject::Type::NONE:
                {
                    break;
                }
                }

            }
        }
    }

    omw->revalidate();
}
