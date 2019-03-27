#ifndef SIMULATIONUICONTROL_H
#define SIMULATIONUICONTROL_H

#include <simulation/SimulationControlListener.h>

class ApplicationControl;
class GeometricData;
class QWidget;
class SceneLeafData;
class SimulationModule;
class SimulationUIWidget;

class SimulationUIControl : public SimulationControlListener
{
public:
    SimulationUIControl(SimulationModule* module,
                        ApplicationControl* ac);

    void init(QWidget* parent);

    QWidget* getWidget();

    void onSimulationButtonToggled(bool checked);
    void onTruncateButtonClicked();

    // Tries to create a fem object for the selected
    // scene node. Calls the corresponding mehtod in
    // the Sim
    void onCreateFEMObjectClicked();
    void onCreateRigidObjectClicked(double mass);
    void onRemoveSimulationObjectClicked();
    void onCreateCollidableClicked();

    void onChangeCollisionRenderingLevel(int level);
    void onEnableCollisionRendering(bool enable);

    SimulationModule* mModule;
    ApplicationControl* mAc;
    SimulationUIWidget* mWidget;

    // SimulationControlListener interface
public:
    virtual void onSimulationObjectAdded(SimulationObject* so);
    virtual void onSimulationObjectRemoved(SimulationObject* so);

    // SimulationControlListener interface
public:
    virtual void onConstraintAdded(SimulationObject* so);
    virtual void onConstraintRemoved(SimulationObject* so);
};

#endif // SIMULATIONUICONTROL_H
