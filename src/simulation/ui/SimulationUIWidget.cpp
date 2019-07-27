#include "SimulationUIControl.h"
#include "SimulationUIWidget.h"
#include "ui_SimulationUIWidget.h"

#include <simulation/SimulationObject.h>
#include <simulation/SimulationObjectVisitor.h>

#include <simulation/constraints/Constraint.h>
#include <simulation/constraints/ConstraintVisitor.h>


SimulationUIWidget::SimulationUIWidget(
        SimulationUIControl* uiControl,
        QWidget* parent)
    : QWidget(parent)
    , mUiControl(uiControl)
    , mUi(new Ui::SimulationUIWidget)
{
    mUi->setupUi(this);

    QButtonGroup* renderCollisionSphereCheckBoxesGroup = new QButtonGroup(this);
    renderCollisionSphereCheckBoxesGroup->addButton(mUi->mCheckBoxRenderLeafs);
    renderCollisionSphereCheckBoxesGroup->addButton(mUi->mCheckBoxRenderAllCollisionSpheres);
}

SimulationUIWidget::~SimulationUIWidget()
{
    delete mUi;
}

void SimulationUIWidget::onSimulationObjectAdded(SimulationObject* simulationObject)
{
    class SOVisitor : public SimulationObjectVisitor
    {
    public:
        SOVisitor()
        {
            name = "unknown type";
        }

        virtual void visit(FEMObject& /*femObject*/)
        {
            name = "FEM Object";
        }

        virtual void visit(SimulationPoint& /*sp*/)
        {
            name = "Simulation Point";
        }

        virtual void visit(RigidBody& /*femObject*/)
        {
            name = "Rigid Body";
        }

        std::string name;
    } visitor;

    simulationObject->accept(visitor);

    // create a QtListWidgetItem and add it to the bidirectional map
    QListWidgetItem* item = new QListWidgetItem(QString::fromStdString(visitor.name),
                                                mUi->mListWidgetSimulationObjects);
    mListWidgetSimulationObjectMap.add(item, simulationObject);
    mUi->mListWidgetSimulationObjects->addItem(item);
}

void SimulationUIWidget::onSimulationObjectRemoved(SimulationObject* simulationObject)
{
    QListWidgetItem* item = mListWidgetSimulationObjectMap.get(simulationObject);
    if (item)
    {
        // remove item from QListWidget
        mUi->mListWidgetSimulationObjects->removeItemWidget(item);
        delete item;
        // remove simulation object and item reference from bidirectional map
        mListWidgetSimulationObjectMap.remove(simulationObject);
    }
}

void SimulationUIWidget::onConstraintAdded(Constraint* constraint)
{
    class SOVisitor : public ConstraintVisitor
    {
    public:
        SOVisitor()
        {
            name = "unknown type";
        }

        virtual void visit(BallJoint* /*ballJoint*/)
        {
            name = "Balljoint";
        }

        virtual void visit(CollisionConstraint* /*cc*/)
        {
            name = "Collision";
        }

        std::string name;
    } visitor;

    constraint->accept(visitor);

    // create a QtListWidgetItem and add it to the bidirectional map
    QListWidgetItem* item = new QListWidgetItem(QString::fromStdString(visitor.name),
                                                mUi->mListWidgetSimulationObjects);
    mListWidgetConstraintMap.add(item, constraint);
    mUi->mListWidgetConstraints->addItem(item);
}

void SimulationUIWidget::onConstraintRemoved(Constraint* constraint)
{
    QListWidgetItem* item = mListWidgetConstraintMap.get(constraint);
    if (item)
    {
        // remove item from QListWidget
        mUi->mListWidgetConstraints->removeItemWidget(item);
        delete item;
        // remove simulation object and item reference from bidirectional map
        mListWidgetConstraintMap.remove(constraint);
    }
}

SimulationObject* SimulationUIWidget::getSelectedSimulationObject()
{
    return mListWidgetSimulationObjectMap.get(mUi->mListWidgetSimulationObjects->currentItem());
}

QtMembersWidget* SimulationUIWidget::getMembersWidget()
{
    return mUi->mMembersWidget;
}

QtMembersWidget* SimulationUIWidget::getFEMObjectMembersWidget()
{
    return mUi->mFEMObjectMembersWidget;
}

void SimulationUIWidget::on_pushButton_toggled(bool checked)
{
    mUiControl->onSimulationButtonToggled(checked);
}

void SimulationUIWidget::on_mButtonTruncate_clicked()
{
    mUiControl->onTruncateButtonClicked();
}

void SimulationUIWidget::on_mCreateFEMObject_clicked()
{
    mUiControl->onCreateFEMObjectClicked();
}

void SimulationUIWidget::on_mRemoveSimulationObject_clicked()
{
    mUiControl->onRemoveSimulationObjectClicked();
}

void SimulationUIWidget::on_mButtonCreateRigidBody_clicked()
{
    // TODO: use a mass of 1 for now.
    mUiControl->onCreateRigidObjectClicked(1);
}

void SimulationUIWidget::on_mSpinBoxCollisionSphereLevel_valueChanged(int /*level*/)
{
    renderCollisionSpheres();
}

void SimulationUIWidget::on_mCheckBoxCollisionSphereLevel_stateChanged(int /*status*/)
{
    renderCollisionSpheres();
}

void SimulationUIWidget::on_mCheckBoxRenderAllCollisionSpheres_stateChanged(int /*status*/)
{
    renderCollisionSpheres();
}

void SimulationUIWidget::renderCollisionSpheres()
{
    if (mUi->mCheckBoxCollisionSphereLevel->checkState() == Qt::Checked)
    {
        // enable ui components
        mUi->mCheckBoxRenderAllCollisionSpheres->setEnabled(true);
        mUi->mSpinBoxCollisionSphereLevel->setEnabled(true);
        mUi->mCheckBoxRenderLeafs->setEnabled(true);

        // enable rendering of BVH
        mUiControl->onEnableCollisionRendering(true);

        // if rendering is wanted
        if (mUi->mCheckBoxRenderAllCollisionSpheres->checkState() == Qt::Checked)
        {
            // if render all
            mUiControl->onChangeCollisionRenderingLevel(-1);
        }
        else if (mUi->mCheckBoxRenderLeafs->checkState() == Qt::Checked)
        {
            mUiControl->onChangeCollisionRenderingLevel(-2);
        }
        else
        {
            // else only render selected level
            mUiControl->onChangeCollisionRenderingLevel(mUi->mSpinBoxCollisionSphereLevel->value());
        }
    }
    else
    {
        // no rendering of collision objects wanted

        // disable ui components
        mUi->mCheckBoxRenderAllCollisionSpheres->setEnabled(false);
        mUi->mSpinBoxCollisionSphereLevel->setEnabled(false);
        mUi->mCheckBoxRenderLeafs->setEnabled(false);

        // diable rendering of BVH
        mUiControl->onEnableCollisionRendering(false);
    }
}

void SimulationUIWidget::on_mCheckBoxRenderLeafs_stateChanged(int /*arg1*/)
{
    renderCollisionSpheres();
}

void SimulationUIWidget::on_mButtonCollidable_clicked()
{
    mUiControl->onCreateCollidableClicked();
}
