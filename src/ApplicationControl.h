#ifndef APPLICATIONCONTROL_H
#define APPLICATIONCONTROL_H

#include <scene/scene_graph/SGControl.h>
#include <scene/scene_graph/SGCore.h>

#include <modules/Module.h>


class DemoLoaderModule;
class InterpolatorModule;
class MeshInterpolationManager;
class RenderModelManager;
class SGUIControl;
class SimulationControl;
class UIControl;

// This class manages all control classes.
// Simply pass an instance to this class in order to allow
// other classes having influence on the application.
// TODO: it should use some kind of dependency maanagement so that
// there will be no conflict when different control classes initialize
// but also depend on the values that are initialized by other control
// classes.
class ApplicationControl
{
public:
    ApplicationControl();
    ~ApplicationControl();

    // initiates controllers for ui and simulation as well
    // as creates a main window
    void initiateApplication();

    // Modules methods
        void createModules();
        // Initiate modules and adds them to ui.
        void initiateModules();
        // Finalizes modules and removes them from ui.
        void finalizeModules();

    // Is called at the start of the rendering step before anything
    // is drawn.
    void handlePreRenderingStep();

    void updateSimulationObjects();

    // Called when the main window is closed.
    void onExit();

    SGControl* getSGControl();
    SGUIControl* getSGUIControl();
    SimulationControl* getSimulationControl();
    UIControl* getUIControl();
    RenderModelManager* getRenderModelManager();
    InterpolatorModule* getInterpolatorModule();
    MeshInterpolationManager* getMeshInterpolationManager();

private:

    // Modules methods
        // Adds for all modules a tab in the ui.
        void addModulesTabs();
        void removeModulesTabs();

    // Getters for all controls
    std::shared_ptr<SGControl> mSGControl;
    std::shared_ptr<SGUIControl> mSGUIControl;
    std::shared_ptr<SimulationControl> mSimulationControl;
    std::shared_ptr<UIControl> mUiControl;
    std::shared_ptr<RenderModelManager> mRenderModelManager;
    std::shared_ptr<MeshInterpolationManager> mMeshInterpolationManager;
    std::shared_ptr<InterpolatorModule> mInterpolatorModule;
    std::vector<std::shared_ptr<Module>> mModules;

    std::shared_ptr<DemoLoaderModule> mDemoLoaderModule;
};

#endif // APPLICATIONCONTROL_H
