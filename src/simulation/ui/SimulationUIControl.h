#ifndef SIMULATIONUICONTROL_H
#define SIMULATIONUICONTROL_H

#include <simulation/SimulationControlListener.h>

#include <ui/selection/SelectionListener.h>

#include <vector>

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

    // Updates the UI member elements for the given scene data.
    // Displays options to adapt the scene data depending on the scene datas
    // type, e.g. if there are FEMObjects and RigidBodies among them, for
    // both fields are displayed in the UI and changes only affect either of
    // them.
    void onSelectedSceneNodesChanged(const std::vector<std::shared_ptr<SceneData>>& sds);

    void updateMemberWidgets(
            const std::vector<std::shared_ptr<SceneData>>& sceneDatas);

//    void updateMemberWidgets(
//            std::iterator<std::random_access_iterator_tag, std::shared_ptr<SceneData>> begin,
//            std::iterator<std::random_access_iterator_tag, std::shared_ptr<SceneData>> end);

    SimulationModule* mModule;
    ApplicationControl* mAc;
    SimulationUIWidget* mWidget;

    // The scene datas that can be accessed with the member widgets in the UI.
    std::vector<std::shared_ptr<SceneData>> mSceneDatasInUI;
};

#endif // SIMULATIONUICONTROL_H
