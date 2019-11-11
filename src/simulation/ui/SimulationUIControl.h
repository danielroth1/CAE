#ifndef SIMULATIONUICONTROL_H
#define SIMULATIONUICONTROL_H

#include <simulation/SimulationControlListener.h>

#include <ui/selection/SelectionListener.h>

class ApplicationControl;
class Constraint;
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
    void onSingleStepButtonClicked();
    void onTruncateButtonClicked();

    // Tries to create a fem object for the selected
    // scene node. Calls the corresponding mehtod in
    // the Simulation.
    void onCreateFEMObjectClicked();
    void onCreateRigidObjectClicked(double mass);
    void onRemoveSimulationObjectClicked();
    void onRemoveConstraintClicked();
    void onCreateCollidableClicked();

    // Prints the corotated stiffness matrices of all selected FEMObjects.
    void onPrintStiffnessMatrixClicked();

    void onChangeCollisionRenderingLevel(int level);
    void onEnableCollisionRendering(bool enable);


    SimulationModule* mModule;
    ApplicationControl* mAc;
    SimulationUIWidget* mWidget;

    // SimulationControlListener interface
public:
    virtual void onSimulationObjectAdded(SimulationObject* so) override;
    virtual void onSimulationObjectRemoved(SimulationObject* so) override;

    virtual void onConstraintAdded(const std::shared_ptr<Constraint>& c) override;
    virtual void onConstraintRemoved(const std::shared_ptr<Constraint>& c) override;

    // SelectionListener interface
public:
    virtual void onSceneNodeSelected(const std::shared_ptr<SceneData>& sd) override;
    virtual void onSelectedSceneNodesChanged(
            const std::set<std::shared_ptr<SceneData> >& sd) override;
    virtual void onSelectedVerticesChanged(
            const std::map<std::shared_ptr<SceneLeafData>, std::vector<ID> >& sv) override;

private:

    // Go over all selected scene nodes and inform the QtMemberWidgets that
    // update the ui elements over changed owners.
    void onSelectedSceneNodesChanged(const std::vector<std::shared_ptr<SceneData>>& sds);
};

#endif // SIMULATIONUICONTROL_H
