#ifndef SIMULATIONUIWIDGET_H
#define SIMULATIONUIWIDGET_H

#include <QWidget>
#include <qlistwidget.h>

#include <data_structures/BidirectionalMap.h>

class Constraint;
class QtMembersWidget;
class QtOwnersMembersWidget;
class SimulationObject;
class SimulationUIControl;

namespace Ui {
class SimulationUIWidget;
}

class SimulationUIWidget : public QWidget
{
    Q_OBJECT

public:
    explicit SimulationUIWidget(
            SimulationUIControl* uiControl,
            QWidget *parent = nullptr);
    ~SimulationUIWidget();



    void onSimulationObjectAdded(SimulationObject* simulationObject);
    void onSimulationObjectRemoved(SimulationObject* simulationObject);

    void onConstraintAdded(Constraint* constraint);
    void onConstraintRemoved(Constraint* constraint);

    // Returns the currently in the QListWidget selected simulation object.
    // If none is selected or there is some inconsistency between this module
    // and the simulation, nulllptr is returned.
    SimulationObject* getSelectedSimulationObject();

    QtMembersWidget* getMembersWidget();

    QtOwnersMembersWidget* getOwnersMembersWidget();

private slots:
    void on_pushButton_toggled(bool checked);

    void on_mButtonTruncate_clicked();

    void on_mCreateFEMObject_clicked();

    void on_mRemoveSimulationObject_clicked();

    void on_mButtonCreateRigidBody_clicked();

    void on_mSpinBoxCollisionSphereLevel_valueChanged(int arg1);

    void on_mCheckBoxCollisionSphereLevel_stateChanged(int arg1);

    void on_mCheckBoxRenderAllCollisionSpheres_stateChanged(int arg1);

    void on_mCheckBoxRenderLeafs_stateChanged(int arg1);

    void on_mButtonCollidable_clicked();

    void on_mPrintStiffnessMatrixButton_clicked();

private:
    void renderCollisionSpheres();

    SimulationUIControl* mUiControl;
    Ui::SimulationUIWidget *mUi;

    BidirectionalMap<QListWidgetItem*, SimulationObject*> mListWidgetSimulationObjectMap;
    BidirectionalMap<QListWidgetItem*, Constraint*> mListWidgetConstraintMap;
    //std::map<SimulationObject*>
};

#endif // SIMULATIONUIWIDGET_H
