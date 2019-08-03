#ifndef SIMULATIONUICONTROL_H
#define SIMULATIONUICONTROL_H

#include <simulation/SimulationControlListener.h>

#include <ui/selection/SelectionListener.h>

class ApplicationControl;
class GeometricData;
class QWidget;
class RenderModelVisitor;
class SceneLeafData;
class SimulationModule;
class SimulationUIWidget;

class SimulationUIControl : public SimulationControlListener,
        public SelectionListener
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
    // the Simulation.
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

    virtual void onConstraintAdded(SimulationObject* so);
    virtual void onConstraintRemoved(SimulationObject* so);

    // SelectionListener interface
public:
    virtual void onSceneNodeSelected(const std::shared_ptr<SceneData>& sd);
    virtual void onSelectedSceneNodesChanged(
            const std::set<std::shared_ptr<SceneData> >& sd);
    virtual void onSelectedVerticesChanged(
            const std::map<std::shared_ptr<SceneLeafData>, std::vector<ID> >& sv);

private:
    void onSelectedSceneNodesChanged(const std::vector<std::shared_ptr<SceneData>>& sds);
};

#endif // SIMULATIONUICONTROL_H
